#!/usr/bin/env python3
"""Mechanical BARR-C:2018 checks for the line and barcode subsystem.

    python3 check_style.py            # checks app_program/line_barcode
    python3 check_style.py FILE ...   # checks the given files

This is NOT a full BARR-C checker.  It catches the mistakes that are easy to
make and easy to see for a machine, so a reviewer can spend time on the rest:

  layout    line length <= 80, no tabs, no trailing blanks, LF line ends,
            file ends with a newline
  files     Doxygen "@file" header, footer "/*** end of file ***/",
            include guard in headers
  braces    every if / else / for / while / do body is a { block }
  safety    no goto, no continue, no "//" comments, no "else if" chain
            without a final "else", every switch has a "default"
  style     constant on the left of == and !=, no bare int/short/long
            (use stdint types), tasks end in _task, handlers in _isr,
            file-scope variables start with g_
  casts     an explicit cast needs a comment on the same or one of the three
            lines above it (a warning, not an error)

Exit status 0 = no errors (warnings are listed but do not fail).
"""

import re
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent
DEFAULT_DIR = ROOT.parent.parent / "app_program" / "line_barcode"

errors = []
warnings = []


def err(path, line, msg):
    errors.append(f"{path.name}:{line}: {msg}")


def warn(path, line, msg):
    warnings.append(f"{path.name}:{line}: {msg}")


def strip_code(text):
    """Return text with comments and string/char literals blanked (same
    length, so columns and line numbers still match)."""
    out = []
    i = 0
    n = len(text)
    while i < n:
        c = text[i]
        two = text[i:i + 2]
        if two == "/*":
            j = text.find("*/", i + 2)
            j = n if j < 0 else j + 2
            out.append("".join(ch if ch == "\n" else " " for ch in text[i:j]))
            i = j
        elif two == "//":
            j = text.find("\n", i)
            j = n if j < 0 else j
            out.append(" " * (j - i))
            i = j
        elif c in "\"'":
            j = i + 1
            while j < n and text[j] != c:
                j += 2 if text[j] == "\\" else 1
            j = min(j + 1, n)
            out.append(c + " " * (j - i - 2) + (c if j - i >= 2 else ""))
            i = j
        else:
            out.append(c)
            i += 1
    result = "".join(out)
    # Preprocessor lines (#if, #else, #define ...) are not statements.
    lines = result.split("\n")
    cont = False
    for idx, line in enumerate(lines):
        if cont or line.lstrip().startswith("#"):
            cont = line.rstrip().endswith("\\")
            lines[idx] = " " * len(line)
    return "\n".join(lines)


def check_layout(path, raw):
    if b"\r" in raw:
        err(path, 1, "CR found: use LF line endings")
    text = raw.decode("utf-8", "replace")
    if not text.endswith("\n"):
        err(path, text.count("\n") + 1, "file does not end with a newline")
    for no, line in enumerate(text.split("\n"), 1):
        if "\t" in line:
            err(path, no, "tab character")
        if len(line) > 80:
            err(path, no, f"line is {len(line)} columns (limit 80)")
        if line != line.rstrip():
            err(path, no, "trailing blank")
    return text


def check_file_frame(path, text):
    if "@file" not in text[:600]:
        err(path, 1, "missing Doxygen @file header")
    if not text.rstrip().endswith("/*** end of file ***/"):
        err(path, text.count("\n"), "missing '/*** end of file ***/' footer")
    if path.suffix == ".h" and "#ifndef" not in text:
        err(path, 1, "header without include guard")


def line_of(code, pos):
    return code.count("\n", 0, pos) + 1


def check_braces(path, code):
    # keyword ( ... ) followed by something other than '{'
    for m in re.finditer(r"\b(if|for|while)\s*\(", code):
        depth, i = 1, m.end()
        while i < len(code) and depth:
            depth += {"(": 1, ")": -1}.get(code[i], 0)
            i += 1
        rest = code[i:].lstrip()
        before = code[:m.start()].rstrip()
        if m.group(1) == "while" and before.endswith("}"):
            continue  # end of do { } while (x);
        if rest.startswith(";") and m.group(1) in ("for", "while"):
            err(path, line_of(code, m.start()), "empty loop body: use { }")
        elif not rest.startswith("{") and not rest.startswith(";"):
            err(path, line_of(code, m.start()),
                f"'{m.group(1)}' body must be a { '{ }' } block")
    for m in re.finditer(r"\belse\b(?!\s*(\{|if\b))", code):
        err(path, line_of(code, m.start()), "'else' body must be a { } block")
    for m in re.finditer(r"\bdo\b(?!\s*\{)", code):
        err(path, line_of(code, m.start()), "'do' body must be a { } block")


def check_chains(path, code):
    """Every 'if ... else if' chain must end in a plain 'else'."""
    for m in re.finditer(r"\belse\s+if\b", code):
        # find the end of this chain by walking forward over blocks
        i = m.start()
        while True:
            j = code.find("{", i)
            if j < 0:
                break
            depth, k = 1, j + 1
            while k < len(code) and depth:
                depth += {"{": 1, "}": -1}.get(code[k], 0)
                k += 1
            rest = code[k:].lstrip()
            if re.match(r"else\s+if\b", rest):
                i = k + (len(code[k:]) - len(rest))
                continue
            if not re.match(r"else\b", rest):
                err(path, line_of(code, m.start()),
                    "'else if' chain does not end with 'else'")
            break


def check_switch(path, code):
    for m in re.finditer(r"\bswitch\s*\(", code):
        j = code.find("{", m.end())
        depth, k = 1, j + 1
        while k < len(code) and depth:
            depth += {"{": 1, "}": -1}.get(code[k], 0)
            k += 1
        if not re.search(r"\bdefault\s*:", code[j:k]):
            err(path, line_of(code, m.start()), "switch without 'default'")


def check_banned(path, code, text):
    for m in re.finditer(r"\bgoto\b", code):
        err(path, line_of(code, m.start()), "goto is not allowed")
    for m in re.finditer(r"\bcontinue\s*;", code):
        err(path, line_of(code, m.start()), "continue is not allowed")
    for no, line in enumerate(text.split("\n"), 1):
        if re.search(r"(^|[^:\"])//", line.split("/*")[0]):
            err(path, no, "'//' comment: use /* */")


def check_types(path, code):
    for m in re.finditer(r"(?<![\w.])(unsigned|short|long)\b(?!\s*\()", code):
        err(path, line_of(code, m.start()),
            f"'{m.group(1)}': use a fixed-width stdint type")
    for m in re.finditer(r"(?<![\w.#])int\s+(\w+)", code):
        if m.group(1) != "main":
            err(path, line_of(code, m.start()),
                "bare 'int': use int32_t or another fixed-width type")


def check_compare(path, code):
    const = r"(?:-?\d[\w.]*|[A-Z][A-Z0-9_]{2,}|'.'|\(\w+\)\s*-?\d\w*)"
    for m in re.finditer(r"([\w\)\]]+)\s*(==|!=)\s*(" + const + r")(?![\w(])",
                         code):
        left = m.group(1)
        if re.fullmatch(const, left) or left.endswith(")"):
            continue
        err(path, line_of(code, m.start()),
            f"put the constant on the left of '{m.group(2)}' "
            f"({m.group(3)} {m.group(2)} {left})")


def check_names(path, code, text):
    for m in re.finditer(r"\b(\w+)\s*\([^;{)]*\)\s*\{", code):
        name = m.group(1)
        if name in ("if", "for", "while", "switch", "main"):
            continue
        if name.endswith("_task") or name.endswith("_isr"):
            continue
        if re.search(r"\b(task|isr)\b", name):
            warn(path, line_of(code, m.start()),
                 f"'{name}': tasks must end in _task, handlers in _isr")
    # file-scope variables: a line at column 0, not a function or macro
    depth = 0
    for no, line in enumerate(code.split("\n"), 1):
        if depth == 0:
            m = re.match(r"^(static\s+)?(volatile\s+)?(const\s+)?"
                         r"(u?int\d+_t|bool|float|double|\w+_t)\s+"
                         r"(\w+)(\[.*\])?\s*(=.*)?;", line)
            if m and not line.startswith(("typedef", "extern", "IMPORT")):
                var = m.group(5)
                if not var.startswith("g_") and not var.startswith("mbf_"):
                    err(path, no, f"file-scope variable '{var}' needs 'g_'")
        depth += line.count("{") - line.count("}")


def check_casts(path, code, text):
    lines = text.split("\n")
    for m in re.finditer(r"\((?:u?int\d+_t|bool|UW|UB|INT|W|ID|TMO|"
                         r"double|float|uint32_t \*)\)\s*[\w(&*-]", code):
        no = line_of(code, m.start())
        window = "\n".join(lines[max(0, no - 7):no])
        in_print = "tm_printf" in "\n".join(lines[max(0, no - 14):no])
        if "/*" not in window and "Cast" not in window and not in_print:
            warn(path, no, "cast without a nearby comment")


def check(path):
    raw = path.read_bytes()
    text = check_layout(path, raw)
    check_file_frame(path, text)
    code = strip_code(text)
    check_braces(path, code)
    check_chains(path, code)
    check_switch(path, code)
    check_banned(path, code, text)
    check_types(path, code)
    check_compare(path, code)
    check_names(path, code, text)
    check_casts(path, code, text)


def main(argv):
    files = [Path(a) for a in argv[1:]]
    if not files:
        files = sorted(list(DEFAULT_DIR.glob("*.c")) +
                       list(DEFAULT_DIR.glob("*.h")))
    for f in files:
        check(f)
    for w in warnings:
        print("warning:", w)
    for e in errors:
        print("error:  ", e)
    print(f"{len(files)} files, {len(errors)} errors, {len(warnings)} warnings")
    return 1 if errors else 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
