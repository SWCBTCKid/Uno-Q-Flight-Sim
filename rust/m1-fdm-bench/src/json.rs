//! Minimal JSON writer (the harness only writes JSON; no parser needed).
use std::fmt::Write;

#[derive(Clone, Debug)]
pub enum J {
    Null,
    Bool(bool),
    I(i64),
    U(u64),
    F(f64),
    S(String),
    A(Vec<J>),
    O(Vec<(String, J)>),
    /// Pre-rendered JSON object members `"k":v,...` (child result fragment).
    Raw(String),
}

pub fn s(x: impl Into<String>) -> J {
    J::S(x.into())
}

pub fn esc(out: &mut String, x: &str) {
    out.push('"');
    for c in x.chars() {
        match c {
            '"' => out.push_str("\\\""),
            '\\' => out.push_str("\\\\"),
            '\n' => out.push_str("\\n"),
            '\r' => out.push_str("\\r"),
            '\t' => out.push_str("\\t"),
            c if (c as u32) < 0x20 => {
                let _ = write!(out, "\\u{:04x}", c as u32);
            }
            c => out.push(c),
        }
    }
    out.push('"');
}

impl J {
    pub fn render(&self, out: &mut String) {
        match self {
            J::Null => out.push_str("null"),
            J::Bool(b) => out.push_str(if *b { "true" } else { "false" }),
            J::I(i) => {
                let _ = write!(out, "{i}");
            }
            J::U(u) => {
                let _ = write!(out, "{u}");
            }
            J::F(f) => {
                if f.is_finite() {
                    // Rust's Display for f64 is the shortest exact round-trip form.
                    let _ = write!(out, "{f}");
                } else {
                    out.push_str("null");
                }
            }
            J::S(x) => esc(out, x),
            J::A(v) => {
                out.push('[');
                for (i, e) in v.iter().enumerate() {
                    if i > 0 {
                        out.push(',');
                    }
                    e.render(out);
                }
                out.push(']');
            }
            J::O(m) => {
                out.push('{');
                let mut first = true;
                for (k, v) in m {
                    if let J::Raw(r) = v {
                        if r.is_empty() {
                            continue;
                        }
                        if !first {
                            out.push(',');
                        }
                        out.push_str(r);
                        first = false;
                        continue;
                    }
                    if !first {
                        out.push(',');
                    }
                    esc(out, k);
                    out.push(':');
                    v.render(out);
                    first = false;
                }
                out.push('}');
            }
            J::Raw(r) => out.push_str(r),
        }
    }

    pub fn to_string(&self) -> String {
        let mut o = String::new();
        self.render(&mut o);
        o
    }

    /// Renders an object's members without the braces (for child fragments).
    pub fn members(&self) -> String {
        let t = self.to_string();
        if t.len() >= 2 && t.starts_with('{') {
            t[1..t.len() - 1].to_string()
        } else {
            String::new()
        }
    }
}
