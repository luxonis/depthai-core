import sys
import subprocess
import re
import tempfile
import os
import textwrap
import shutil

# Usage
if len(sys.argv) < 4:
    print(f"Usage: {sys.argv[0]} [module_name] [library_dir] [pip_temp_lib_folder]")
    exit(-1)

MODULE_NAME = sys.argv[1]
DIRECTORY = sys.argv[2]
PIP_TEMP_LIB_FOLDER = sys.argv[3]

print(f'Generating stubs for module: "{MODULE_NAME}" in directory: "{DIRECTORY}"')
print(f'PIP_TEMP_LIB_FOLDER: "{PIP_TEMP_LIB_FOLDER}"')

try:

    # On Windows, copy files from the lib folder to the temp build folder
    # so that they are co-located with the bindings module and 
    # stubgen can find all the necessary libraries the depthai links against.
    # Note that this is not an issue on Linux and macOS.
    stubgen_root = DIRECTORY
    if sys.platform == 'win32':
        for item in os.listdir(DIRECTORY):
            src = os.path.join(DIRECTORY, item)
            dst = os.path.join(PIP_TEMP_LIB_FOLDER, item)
            shutil.copytree(src, dst, dirs_exist_ok=True) if os.path.isdir(src) else shutil.copy2(src, dst)
        stubgen_root = PIP_TEMP_LIB_FOLDER

    # Add __init__.py
    with open(f'{DIRECTORY}/__init__.py', 'w') as file:
        content = textwrap.dedent('''
            from depthai import _cxxdepthai
            __all__ = []
            __obj = None
            for __name in dir(_cxxdepthai):
                if __name.startswith("_"):
                    continue
                __obj = getattr(_cxxdepthai, __name)
                try:
                    setattr(__obj, "__module__", __name__) # depthai._cxxdepthai.X -> depthai.X
                except AttributeError:
                    pass
                __all__.append(__name)
                globals()[__name] = __obj
            del __name, __obj, _cxxdepthai
        ''')
        file.write(content)

    # Create stubs, add PYTHONPATH to find the build module
    # CWD to to extdir where the built module can be found to extract the types
    env = os.environ
    env['PYTHONPATH'] = f'{stubgen_root}{os.pathsep}{env.get("PYTHONPATH", "")}'

    # Prevent __pycache__ generation on the import bellow
    sys.dont_write_bytecode = True 

    # Test importing depthai after PYTHONPATH is specified
    try:
        import _cxxdepthai
    except Exception as ex:
        print(f'Could not import depthai: {ex}')

    print(f'PYTHONPATH set to {env["PYTHONPATH"]}')
    # Check if stubgen has the `--include-docstrings` flag
    includeDocstrings = False
    output = subprocess.check_output(['stubgen', '--help'], env=env)
    if b'--include-docstrings' in output:
        includeDocstrings = True
        print("Will include docstrings in stubs")
    else:
        print("Will not include docstrings in stubs")

    with tempfile.TemporaryDirectory() as temp_dir:

        # Generate stubs in a temporary directory
        parameters = ['stubgen', '-p', "_cxxdepthai", '-o', os.path.join(temp_dir)]
        if includeDocstrings:
            parameters.insert(1, '--include-docstrings')
        subprocess.check_call(parameters, cwd=stubgen_root, env=env)

        # Stubgen will put the stubs in the temp_dir/depthai folder
        # We need to copy the stubs to the main directory
        src_stub_dir, dst_stub_dir = os.path.join(temp_dir, "_cxxdepthai"), DIRECTORY
        if os.path.exists(src_stub_dir):
            os.makedirs(dst_stub_dir, exist_ok=True)
            for item in os.listdir(src_stub_dir):
                s = os.path.join(src_stub_dir, item)
                d = os.path.join(dst_stub_dir, item)
                shutil.copytree(s, d, dirs_exist_ok=True) if os.path.isdir(s) else shutil.copy2(s,d)

    # Add py.typed
    open(f'{DIRECTORY}/py.typed', 'a').close()

    # imports and overloads
    with open(f'{DIRECTORY}/__init__.pyi' ,'r+') as file:
        # Read
        contents = file.read()

        # Add imports
        stubs_import = textwrap.dedent('''
            # Ensures that the stubs are picked up - thanks, numpy project
            import typing
            json = dict
            from pathlib import Path
            from typing import Set, Type, TypeVar
            T = TypeVar('T')
        ''') + contents

        # Create 'create' overloads
        nodes = re.findall('def \\S*\\(self\\) -> node.\\(\\S*\\):', stubs_import)
        overloads = ''
        for node in nodes:
            overloads = overloads + f'\\1@overload\\1def create(self, arg0: typing.Type[node.{node}]) -> node.{node}: ...'
        final_stubs = re.sub(r"([\s]*)def create\(self, arg0: object\) -> Node: ...", f'{overloads}', stubs_import)

        final_lines = []
        for line in final_stubs.split('\n'):
            if 'def create(self, arg0: object, *args, **kwargs) -> Node:' in line:
                if includeDocstrings:
                    ending = 'T:'
                else:
                    ending = 'T: ...'
                final_lines.append(f"    def create(self, arg0: Type[T], *args, **kwargs) -> {ending}")
                continue

            # Fix the issue where the stubs would have `import MessageQueue`
            # at the top of the file. This caused `dai.MessageQueue` not to be 
            # especially IDE friendly.
            if line.strip() == 'import MessageQueue':
                continue

            final_lines.append(line)

        final_stubs = '\n'.join(final_lines)
        # Writeout changes
        file.seek(0)
        file.truncate(0)
        file.write(final_stubs)

    # node fixes
    with open(f'{DIRECTORY}/node/__init__.pyi' ,'r+') as file:
        # Read
        contents = file.read()

        # Add imports
        stubs_import = textwrap.dedent('''
            from pathlib import Path
            from typing import Set
        ''') + contents

        # Remove import depthai.*
        final_stubs = re.sub(r"import depthai\.\S*", "", stubs_import)

        for line in final_stubs.split('\n'):
            final_lines.append(line)
            if "class HostNode(ThreadedHostNode):" in line:
                final_lines.append('    def link_args(self, *args) -> None: ...')
                final_lines.append('    def __init__(self, *args) -> None: ...')

        final_stubs = '\n'.join(final_lines)
        # Writeout changes
        file.seek(0)
        file.truncate(0)
        file.write(final_stubs)

    # Replace all occurrences of _cxxdepthai with depthai._cxxdepthai in all .pyi files
    for root, dirs, files in os.walk(f'{DIRECTORY}'):
        for file_name in files:
            if file_name.endswith('.pyi'):
                file_path = os.path.join(root, file_name)
                with open(file_path, 'r+') as f:
                    contents = f.read()
                    new_contents = contents.replace('_cxxdepthai', 'depthai._cxxdepthai')
                    if new_contents != contents:
                        f.seek(0)
                        f.truncate(0)
                        f.write(new_contents)

    # Flush previous stdout
    sys.stdout.flush()

    # Check syntax (Mypy and later Pylance/Pyright)
    # Windows limitation - another process cannot normally read temporary file that is opened by this process
    # Close first and delete manually afterwards
    try:
        config = tempfile.NamedTemporaryFile(mode='w', delete=False)
        config.write('[mypy]\nignore_errors = True\n')
        config.close()
        print(f'Mypy config file: {config.name}')
        # Mypy check
        subprocess.check_call([sys.executable, '-m' 'mypy', f'{DIRECTORY}', f'--config-file={config.name}'], env=env)
    finally:
        os.unlink(config.name)
    # # TODO(thamarpe) - Pylance / Pyright check
    # subprocess.check_call([sys.executable, '-m' 'pyright', f'{DIRECTORY}/{MODULE_NAME}'], env=env)

    def process_init_pyi(file_path, is_depthai_root=False):
        # Read old __init__.pyi
        with open(file_path, 'r+') as file:
            contents = file.read()

        # Prepare imports
        dir_path = os.path.dirname(file_path)

        modules = list()
        # Find all .pyi files and directories
        for f in os.listdir(dir_path):
            if f.endswith('.pyi') and f != '__init__.pyi':
                modules.append(f)
            elif os.path.isdir(os.path.join(dir_path, f)):
                modules.append(f)

        prefix = "from depthai import " if is_depthai_root else "from . import "
        imports = '\n'.join([f'{prefix}{os.path.splitext(m)[0]} as {os.path.splitext(m)[0]}' for m in modules])

        # Add imports to old __init__.pyi
        new_contents = imports + '\n' + contents

        # Writeout changes
        with open(file_path, 'w') as file:
            file.write(new_contents)

    # Process all __init__.pyi files
    for root, dirs, files in os.walk(f'{DIRECTORY}'):
        is_depthai_root = (root == f'{DIRECTORY}')
        if os.path.exists(os.path.join(root, '__init__.pyi')):
            process_init_pyi(os.path.join(root, '__init__.pyi'), is_depthai_root)

except subprocess.CalledProcessError as err:
    print(f"Error during stub generation: {err}")
    exit(err.returncode)

except Exception as e:
    print(f"An error occurred: {e}")
    exit(-1)


exit(0)
