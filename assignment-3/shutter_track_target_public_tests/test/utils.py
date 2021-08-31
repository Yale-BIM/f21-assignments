import os

def compute_import_path(*args):
    """
    Helper function to compute import path for relative imports
    """
    test_dir = os.path.dirname(os.path.abspath(__file__))
    import_dir = os.path.abspath(os.path.join(test_dir, '..', '..'))
    for path in args:
        import_dir = os.path.abspath(os.path.join(import_dir, path))
    return import_dir
