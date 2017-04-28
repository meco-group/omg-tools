import os


def test_generator():
    if 'NODE_TOTAL' or 'NODE_INDEX' not in os.environ:
        node_total = 1
        node_index = 0
    else:
        node_total = int(os.environ['NODE_TOTAL'])
        node_index = int(os.environ['NODE_INDEX'])
    example_dir = os.path.join(os.getcwd(), 'examples')
    files = os.listdir(example_dir)
    example_files = []
    for f in files:
        if os.path.isfile(os.path.join(example_dir, f)) and f.endswith('.py'):
            example_files.append(f)
    n_files = len(example_files)/node_total
    if node_index == node_total-1:
        test_files = example_files[node_index*n_files:]
    else:
        test_files = example_files[node_index*n_files:(node_index+1)*n_files]
    for f in test_files:
        yield run_example, f


def run_example(filename):
    example_dir = os.path.join(os.getcwd(), 'examples')
    print ''
    os.system('python %s' % os.path.join(example_dir, filename))
