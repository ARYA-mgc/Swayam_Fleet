import os
import tokenize
import io

def strip_comments(file_path):
    with open(file_path, 'r', encoding='utf-8') as f:
        source = f.read()

    # Keep header: lines at the start that begin with '#'
    lines = source.split('\n')
    header_lines = []
    for line in lines:
        if line.strip().startswith('#') or line.strip() == '':
            header_lines.append(line)
        else:
            break
            
    # Now use tokenize to strip all comments from the entire source
    tokens = tokenize.generate_tokens(io.StringIO(source).readline)
    out_tokens = []
    for tok in tokens:
        if tok.type == tokenize.COMMENT:
            continue
        out_tokens.append(tok)
        
    unparsed = tokenize.untokenize(out_tokens)
    
    # But untokenize keeps the original newlines where comments were, which leaves blank lines.
    # Let's clean up multiple blank lines.
    cleaned_lines = [line for line in unparsed.split('\n') if line.strip() or line == '']
    
    # We want to remove all comments, then put the header back.
    # Wait, the tokenize output already includes the header if we just didn't skip them.
    # Let's rebuild the file: header + the rest (without comments).
    
    # Alternative: Just parse AST and unparse it. That's the safest and cleanest way to strip comments and normalize formatting.
    import ast
    try:
        tree = ast.parse(source)
        # Remove docstrings
        for node in ast.walk(tree):
            if isinstance(node, (ast.FunctionDef, ast.ClassDef, ast.AsyncFunctionDef, ast.Module)):
                if ast.get_docstring(node):
                    node.body.pop(0)
        clean_code = ast.unparse(tree)
    except Exception as e:
        print(f"Failed to parse {file_path}: {e}")
        return
        
    final_source = "\n".join(header_lines) + "\n" + clean_code + "\n"
    
    with open(file_path, 'w', encoding='utf-8') as f:
        f.write(final_source)
    print(f"Stripped {file_path}")

for root, _, files in os.walk(r'd:\sawayam_drone\src\swayam'):
    for file in files:
        if file.endswith('.py'):
            strip_comments(os.path.join(root, file))
