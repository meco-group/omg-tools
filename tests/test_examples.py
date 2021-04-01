from __future__ import division
import os
import imp
import subprocess

def test_generator():
    if 'NODE_TOTAL' not in os.environ or 'NODE_INDEX' not in os.environ:
        node_total = 1
        node_index = 0
    else:
        node_total = int(os.environ['NODE_TOTAL'])
        node_index = int(os.environ['NODE_INDEX'])
    example_dir = os.path.join(os.getcwd(), 'examples')
    files = os.listdir(example_dir)
    example_files = []
    for f in files:
        if (os.path.isfile(os.path.join(example_dir, f)) and f.endswith('.py') and not 'gui' in f):
            example_files.append(f)
    example_files.sort()
    n_files = len(example_files)//node_total
    if node_index == node_total-1:
        test_files = example_files[node_index*n_files:]
    else:
        test_files = example_files[node_index*n_files:(node_index+1)*n_files]
    for f in test_files:
        yield run_example, f


def test_export():
    files = {'export': 'Point2Point',
             'export_f': 'FormationPoint2Point',
             'export_r': 'RendezVous'}
    for d, f in files.items():
        if os.path.isdir(os.path.join(os.getcwd(), d)):
            print(subprocess.check_output(
                ("cd %s && make && cd bin && ./%s && cd ../..") %
                (d, f), shell=True, stderr=subprocess.STDOUT))


def run_example(filename):
    subprocess.Popen("rm solver.*", shell=True).wait()
    example_dir = os.path.join(os.getcwd(), 'examples')
    name = filename.split('.')[-2]
    print(name)
    try:
      imp.load_source(name,
                    os.path.join(example_dir, filename))

    #  
    # Forgive sys.exit(0)
    except SystemExit as e:
      if e.code!=0:
        import sys
        sys.exit(e.code)
    finally:
      subprocess.Popen("mkdir dumps/%s && mv solver.* dumps/%s" % (name, name),shell=True).wait()
      pass

