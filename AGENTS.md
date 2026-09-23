Minimal Code

- Write the absolute minimum code necessary. No more, no less.
- Reuse existing code before creating new
- No speculative features, abstractions (wait for 3+ uses), or config "for flexibility"
- Keep inline until used in 2+ places, or extraction clearly improves readability
- No one-off wrappers, helpers, or single-consumer types/interfaces
- Delete unused code; don't comment out "for later"
- Question every line: is it necessary? Does it already exist?
- Red flags: single-use components, one-call helpers, excess comments, premature utils/helpers/common files.