use std::fmt;

/// Error indicating that a query is not supported between certain shapes
#[derive(Debug, Copy, Clone, Eq, PartialEq)]
pub struct Unsupported;

impl fmt::Display for Unsupported {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.pad("query not supported between these shapes")
    }
}

impl std::error::Error for Unsupported {}
