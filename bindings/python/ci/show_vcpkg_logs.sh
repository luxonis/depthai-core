echo "Searching vcpkg-manifest-install.log for failing port logs..."
LOGFILE=$(find . -name "vcpkg-manifest-install.log" | head -n 1)
if [ -z "$LOGFILE" ]; then
    echo "No vcpkg-manifest-install.log found!"
    exit 1
else
    echo "Found log file: $LOGFILE"
fi

# 1) Grab lines following "See logs for more information:"
# 2) From those lines, extract only the file paths ending in ".log"
# 3) Remove any leading spaces
grep -A50 "See logs for more information:" $LOGFILE \
    | grep "\.log" \
    | sed 's/^[[:space:]]*//' \
    > failed_logs.txt

# Check if we found any logs
# If not, print a message and exit
# Otherwise, print the log paths
if [ ! -s failed_logs.txt ]; then
    echo "No failed logs found!"
    exit 1
else
    echo "Found the following failed logs:"
    cat failed_logs.txt
fi

# Now read each log path we found, and print it
while IFS= read -r log; do
    echo "==== Showing log: $log ===="
    cat "$log" || true
echo

# Also show CMakeOutput.log (and/or CMakeError.log) within the same port’s directory
# Often it’s in the same or neighboring folder (e.g. arm64-linux-rel/CMakeFiles/CMakeOutput.log).
# We'll "walk up" one directory and search for CMakeOutput.log in CMakeFiles/.
port_dir="$(dirname "$log")"
# In some cases (e.g. config-arm64-linux-out.log), you may need to go up another level:
# port_dir="$(dirname "$port_dir")"

# We'll now look for any CMakeOutput.log within this port’s subdirectories
found_outputs=$(find "$port_dir" -name "CMakeOutput.log" -o -name "CMakeError.log" -print 2>/dev/null)
if [ -n "$found_outputs" ]; then
    for cof in $found_outputs; do
        echo "==== Showing $cof ===="
        cat "$cof" || true
        echo
    done
fi
done < failed_logs.txt
