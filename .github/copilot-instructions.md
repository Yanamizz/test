# Project Guidelines

## Decision Making
- State assumptions explicitly. If something is ambiguous, ask before changing code.
- Prefer the simplest solution that satisfies the request.
- Do not add speculative abstractions, features, or error handling.

## Code Changes
- Touch only what is required.
- Match existing style and preserve public APIs unless the task requires a change.
- Remove only imports, variables, or functions introduced by the change that become unused.
- If unrelated dead code is noticed, mention it instead of deleting it.

## Verification
- Define a verifiable success criterion before coding.
- Prefer the smallest test or check that proves the change.
- Loop until the requested behavior is verified.
