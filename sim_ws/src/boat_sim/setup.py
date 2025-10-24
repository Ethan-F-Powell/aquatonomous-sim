from setuptools import setup
from glob import glob
import os

package_name = 'boat_sim'
share = os.path.join('share', package_name)

def collect_tree(root):
    pairs = []
    if os.path.isdir(root):
        for d, _, files in os.walk(root):
            if files:
                pairs.append((os.path.join(share, d),
                              [os.path.join(d, f) for f in files]))
    return pairs

data_files = [
    ('share/ament_index/resource_index/packages', ['resource/boat_sim']),
    (share, ['package.xml']),
    (os.path.join(share, 'launch'), glob('launch/*.py')),
    (os.path.join(share, 'worlds'), glob('worlds/*.sdf')),
]
data_files += collect_tree('models')
data_files += collect_tree('config')

setup(
    name=package_name,
    version='0.0.1',
    packages=[],
    data_files=data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='you',
    maintainer_email='you@example.com',
    description='Single-package simulator for the boat',
    license='MIT',
)
