---
name: pr-review
description: Use when reviewing pull requests or judging whether a change is ready to merge in depthai-core. Focuses on concrete findings first, with emphasis on style consistency, test coverage, regression protection, and public API documentation.
---

# PR Review

Review with a findings-first mindset. Prioritize behavioral risk, missing validation, and API/documentation gaps over cosmetic commentary.

## Review Checklist

### Backwards compatibility
- Check whether the change is additive or if it modifies existing behavior.
- If it modifies existing behavior, check whether the change is clearly called out in the description and whether the risk level is acknowledged.
- If the change is breaking, check whether it is justified and whether the description clearly outlines the breaking change and its impact.

### Architectural consistency
- Check whether the change fits the architectural patterns of the touched area
   - For example, if a change refactors the general `Node.hpp` structure, check whether the same could be done with the existing `Node` class, the existing patterns in the repository.
- The critical concepts that define the architecture are:
   - `Device.hpp` `Pipeline.hpp`  and `Node.hpp`

### Style
- Check whether naming follows existing camelCase conventions in the touched area.
- Check whether formatting tools such as clang-format should have been run.
- Avoid introducing `_` prefixes or suffixes for members when the surrounding codebase does not use them.

### Tests
- New features should have at least basic coverage.
- Bug fixes should include a regression test when practical.
- Call out when validation is missing or weaker than the risk level of the change
- The tests should be marked to run on `ci` whenever possible

### Public Surface
- Check whether public APIs have documentation comments where expected.
- Check whether examples or user-facing docs should change with the API.

## Review Output

Lead with findings ordered by severity. Keep summaries brief and secondary.
