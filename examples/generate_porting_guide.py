import os
from pathlib import Path

CPP_DIR = Path("cpp")
PY_DIR = Path("python")
OUTPUT_FILE = "PORTING_GUIDE.md"

def find_examples(base_dir, extension):
    examples = {}
    for dirpath, _, files in os.walk(base_dir):
        for f in files:
            if f.endswith(extension):
                relative_path = Path(dirpath).relative_to(base_dir)
                key = f"{relative_path}/{f.removesuffix(extension)}"
                examples[key.replace("\\", "/")] = Path(dirpath) / f
    return examples

def read_file(path):
    try:
        return path.read_text(encoding="utf-8")
    except Exception as e:
        return f"<error reading file: {e}>"

def main():
    cpp_files = find_examples(CPP_DIR, ".cpp")
    py_files = find_examples(PY_DIR, ".py")

    # Only keep keys that are present in both
    common_keys = sorted(set(cpp_files.keys()) & set(py_files.keys()))

    with open(OUTPUT_FILE, "w", encoding="utf-8") as out:
        # Preface
        out.write("# Porting Guide: Python ⇔ C++ Examples\n\n")
        out.write(
            "This document shows paired examples that exist **both** in Python and C++.\n"
            "It is intended to help with understanding how equivalent logic is implemented across the two languages.\n\n"
        )

        for key in common_keys:
            section_name = key.split("/")
            category = section_name[0] if len(section_name) > 1 else "Top-level"
            example = section_name[-1]

            out.write(f"## {category}: {example}\n\n")

            py_path = py_files[key]
            cpp_path = cpp_files[key]

            out.write(f"### {py_path}\n\n```python\n{read_file(py_path)}\n```\n\n")
            out.write(f"### {cpp_path}\n\n```cpp\n{read_file(cpp_path)}\n```\n\n")

    print(f"[✓] Porting guide written to {OUTPUT_FILE} with {len(common_keys)} example pairs")

if __name__ == "__main__":
    main()
