from setuptools import setup, find_packages

version="2025.0.0.1"
# remember to create a requirements.txt
# pipreqs makes this easier
def get_requirements():
    """
    Parse dependencies from 'requirements.in' file.

    Collecting dependencies from 'requirements.in' as a list,
    this list will be used by 'install_requires' to specify minimal dependencies
    needed to run the application.
    """
    with open('requirements.txt') as fd:
        return [l.split("==")[0] for l in fd.read().splitlines()]

install_requires = get_requirements()

setup(
    name='FROGlib',
    version=version,
    description='FROG 3160 library of commonly used classes and functions',
    author='FROG - Team 3160',
    author_email='frog3160web.gmail.com',
    packages=find_packages(),
    install_requires=install_requires,
    classifiers=[
        'Programming Language :: Python :: 3',
        'Operating System :: OS Independent',
    ],
)
