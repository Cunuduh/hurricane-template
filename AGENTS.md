## hurricane-template build command
* Use `cargo build --release` to build the project. Very simple. No need to specify target.
* Use `cargo check` to check for errors super quickly.

## code style
* Do not document functions for me. I will do that myself.
* Only write comments when the code is not clear enough and absolutely necessary (for example explaining complex math or justifying magic numbers).
    * Avoid obvious comments like `// increment i` or `// return the value`.
    * Do not write comments describing the changes you made to the code.
    * The comments should be in lowercase and not end with a period.
* No excessive error handling with many branches and early returns.