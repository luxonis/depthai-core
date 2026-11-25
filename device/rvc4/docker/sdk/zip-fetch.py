#!/usr/bin/env python3
"""Extract members of a remote zip archive with HTTP range requests, without a download of the complete archive.

usage: zip-fetch.py <url> <destination dir> <member pattern>...
A pattern is a shell pattern on the member name ('*' also matches '/').
Exit status: 0 = done, 4 = the server does not answer range requests, 1 = other error.
"""
import fnmatch
import io
import sys
import time
import urllib.error
import urllib.request
import zipfile

TAIL = 2 << 20  # the end of a zip archive has its list of members: read it in one request
JOIN = 256 << 10  # members with less than this distance between them come in one request
BLOCK = 1 << 20  # read-ahead for a read outside the planned ranges


class RangeNotSupported(Exception):
    pass


class HttpFile(io.RawIOBase):
    """A read-only, seekable file on an HTTP server. It keeps each byte range that it downloaded."""

    def __init__(self, url):
        self.url, self.pos, self.fetched, self.requests, self.segments = url, 0, 0, 0, []
        status, headers, _ = self._get(0, 0)
        content_range = headers.get("Content-Range", "")
        if status != 206 or "/" not in content_range:
            raise RangeNotSupported()
        self.size = int(content_range.rsplit("/", 1)[1])
        self.load(max(0, self.size - TAIL), self.size)

    def _get(self, first, last):
        request = urllib.request.Request(self.url, headers={"Range": f"bytes={first}-{last}"})
        for attempt in range(4):
            try:
                with urllib.request.urlopen(request, timeout=120) as response:
                    data = response.read()
                    self.url = response.geturl()  # the next request goes directly to the server after a redirect
                    self.requests += 1
                    self.fetched += len(data)
                    return response.status, response.headers, data
            except urllib.error.HTTPError:
                raise
            except (urllib.error.URLError, OSError):
                if attempt == 3:
                    raise
                time.sleep(2 * (attempt + 1))

    def load(self, start, end):
        """Download the bytes start..end-1 that no earlier request has."""
        for have, data in self.segments:
            if have <= start < have + len(data):
                start = have + len(data)
            if have < end <= have + len(data):
                end = have
        if start >= end:
            return
        status, _, data = self._get(start, end - 1)
        if status != 206 or len(data) != end - start:
            raise RangeNotSupported()
        self.segments.append((start, data))

    def readable(self):
        return True

    def seekable(self):
        return True

    def tell(self):
        return self.pos

    def seek(self, offset, whence=io.SEEK_SET):
        self.pos = {io.SEEK_SET: 0, io.SEEK_CUR: self.pos, io.SEEK_END: self.size}[whence] + offset
        return self.pos

    def readinto(self, buffer):
        count = min(len(buffer), self.size - self.pos)
        if count <= 0:
            return 0
        for _ in range(2):
            for start, data in self.segments:
                if start <= self.pos < start + len(data):
                    count = min(count, start + len(data) - self.pos)
                    buffer[:count] = data[self.pos - start : self.pos - start + count]
                    self.pos += count
                    return count
            self.load(self.pos, self.pos + count)
        raise RangeNotSupported()


def main():
    if len(sys.argv) < 4:
        sys.exit(__doc__)
    url, destination, patterns = sys.argv[1], sys.argv[2], sys.argv[3:]
    try:
        remote = HttpFile(url)
        with zipfile.ZipFile(io.BufferedReader(remote, BLOCK)) as archive:
            entries = sorted(archive.infolist(), key=lambda m: m.header_offset)
            # A member ends where the next one starts. The last one ends at the list of members.
            ends = [m.header_offset for m in entries[1:]] + [archive.start_dir]
            members = [(m, end) for m, end in zip(entries, ends) if not m.is_dir() and any(fnmatch.fnmatchcase(m.filename, p) for p in patterns)]
            ranges = []
            for member, end in members:
                if ranges and member.header_offset - ranges[-1][1] <= JOIN:
                    ranges[-1][1] = end
                else:
                    ranges.append([member.header_offset, end])
            for start, end in ranges:
                remote.load(start, end)
            for member, _ in members:
                archive.extract(member, destination)  # extract() checks the CRC-32 of each member
    except RangeNotSupported:
        print("zip-fetch: the server does not answer range requests", file=sys.stderr)
        sys.exit(4)
    except (urllib.error.URLError, OSError, zipfile.BadZipFile) as error:
        print(f"zip-fetch: {error}", file=sys.stderr)
        sys.exit(1)
    if not members:
        print("zip-fetch: no member of the archive has one of the names", file=sys.stderr)
        sys.exit(1)
    print(f"{len(members)} files, {remote.fetched / 1e6:.1f} MB of {remote.size / 1e6:.0f} MB downloaded in {remote.requests} requests")


if __name__ == "__main__":
    main()
