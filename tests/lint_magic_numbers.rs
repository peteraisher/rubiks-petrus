//! Lint: no magic numbers in library/binary code.
//!
//! Clippy has no magic-number lint, so this test enforces the rule: outside a
//! `const`, `static` or `type` item, an integer literal must be 0, 1 or 2. Anything
//! larger needs a named constant. Comments, strings and char literals are
//! ignored. A line may opt out with a trailing `// magic-ok: <reason>`.

use std::fs;
use std::path::Path;

const MAX_PLAIN_LITERAL: u64 = 2;

fn rust_files(dir: &Path, out: &mut Vec<std::path::PathBuf>) {
    for entry in fs::read_dir(dir).unwrap() {
        let p = entry.unwrap().path();
        if p.is_dir() {
            rust_files(&p, out);
        } else if p.extension().is_some_and(|e| e == "rs") {
            out.push(p);
        }
    }
}

/// Blank out comments, string and char literals, keeping line structure.
fn strip(src: &str) -> String {
    let b = src.as_bytes();
    let mut out = String::with_capacity(b.len());
    let mut i = 0;
    while i < b.len() {
        let c = b[i];
        if c == b'/' && b.get(i + 1) == Some(&b'/') {
            while i < b.len() && b[i] != b'\n' {
                i += 1;
            }
        } else if c == b'/' && b.get(i + 1) == Some(&b'*') {
            while i < b.len() && !(b[i] == b'*' && b.get(i + 1) == Some(&b'/')) {
                if b[i] == b'\n' {
                    out.push('\n');
                }
                i += 1;
            }
            i += 2;
        } else if c == b'"' {
            i += 1;
            while i < b.len() && b[i] != b'"' {
                if b[i] == b'\\' {
                    i += 1;
                }
                if b[i] == b'\n' {
                    out.push('\n');
                }
                i += 1;
            }
            i += 1;
            out.push_str("\"\"");
        } else if c == b'\'' && b.get(i + 2) == Some(&b'\'') {
            i += 3; // 'x'
            out.push_str("' '");
        } else if c == b'\'' && b.get(i + 1) == Some(&b'\\') {
            i += 1;
            while i < b.len() && b[i] != b'\'' {
                i += 1;
            }
            i += 1;
            out.push_str("' '");
        } else {
            out.push(c as char);
            i += 1;
        }
    }
    out
}

fn literal_value(tok: &str) -> Option<u64> {
    let t = tok.replace('_', "");
    if t.contains('.') || t.contains('e') && !t.starts_with("0x") {
        return None; // float
    }
    let (digits, radix) = match t.strip_prefix("0x") {
        Some(h) => (h.to_string(), 16),
        None => (t.clone(), 10),
    };
    let end = digits.find(|c: char| !c.is_digit(radix)).unwrap_or(digits.len());
    u64::from_str_radix(&digits[..end], radix).ok()
}

/// (line number, literal) for each violation in `src`.
fn violations(src: &str) -> Vec<(usize, String)> {
    let raw_lines: Vec<&str> = src.lines().collect();
    let stripped = strip(src);
    let mut out = Vec::new();
    let mut in_const = false;
    let mut depth: i32 = 0;
    for (n, line) in stripped.lines().enumerate() {
        let t = line.trim_start();
        let t = t.strip_prefix("pub(crate) ").or_else(|| t.strip_prefix("pub ")).unwrap_or(t);
        let starts_item = (t.starts_with("const ") && !t.starts_with("const fn"))
            || t.starts_with("static ")
            || t.starts_with("type ");
        if !in_const && starts_item {
            in_const = true;
            depth = 0;
        }
        let allowed = in_const
            || t.starts_with("#[")
            || raw_lines.get(n).is_some_and(|l| l.contains("// magic-ok:"));
        let b = line.as_bytes();
        let mut i = 0;
        while i < b.len() {
            let c = b[i] as char;
            match c {
                '{' | '(' | '[' => depth += 1,
                '}' | ')' | ']' => depth -= 1,
                ';' if in_const && depth <= 0 => in_const = false,
                _ => {}
            }
            let prev_ident = i > 0 && (b[i - 1].is_ascii_alphanumeric() || b[i - 1] == b'_');
            if c.is_ascii_digit() && !prev_ident {
                let start = i;
                while i < b.len() && (b[i].is_ascii_alphanumeric() || b[i] == b'_' || b[i] == b'.')
                {
                    // stop at a range `..` or method call on the literal
                    if b[i] == b'.' && !b.get(i + 1).is_some_and(|d| d.is_ascii_digit()) {
                        break;
                    }
                    i += 1;
                }
                let tok = &line[start..i];
                let after_dot = start > 0 && b[start - 1] == b'.'; // tuple field `.0`
                if !allowed && !after_dot {
                    if let Some(v) = literal_value(tok) {
                        if v > MAX_PLAIN_LITERAL {
                            out.push((n + 1, tok.to_string()));
                        }
                    }
                }
                continue;
            }
            i += 1;
        }
    }
    out
}

#[test]
fn lint_self_check() {
    let bad = "fn f() { let x = 3; }\n";
    assert_eq!(violations(bad).len(), 1);
    let ok = "const N: usize = 54;\nfn f() -> u8 { let s = \"99\"; 2 } // 77\nstatic T: [u8; 4] = [\n 5, 6,\n];\n";
    assert!(violations(ok).is_empty(), "{:?}", violations(ok));
    assert!(violations("fn f() { g(9) } // magic-ok: test\n").is_empty());
    assert!(violations("fn f() { a.0 + vld1q_u8_x2(p) }\n").is_empty());
}

#[test]
fn no_magic_numbers_in_src() {
    let mut files = Vec::new();
    rust_files(&Path::new(env!("CARGO_MANIFEST_DIR")).join("src"), &mut files);
    files.sort();
    let mut report = String::new();
    for f in &files {
        let src = fs::read_to_string(f).unwrap();
        for (line, tok) in violations(&src) {
            report.push_str(&format!("{}:{line}: literal {tok}\n", f.display()));
        }
    }
    assert!(report.is_empty(), "magic numbers (name them as constants):\n{report}");
}
