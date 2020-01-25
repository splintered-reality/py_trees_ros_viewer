#!/usr/bin/env python3

import os

from distutils import log
from setuptools import find_packages, setup
from setuptools.command.develop import develop
from setuptools.command.install import install

package_name = 'py_trees_ros_viewer'


# This is somewhat dodgy as it will escape any override from, e.g. the command
# line or a setup.cfg configuration. It does however, get us around the problem
# of setup.cfg influencing requirements install on rtd installs
#
# TODO: should be a way of detecting whether scripts_dir has been influenced
# from outside
def redirect_install_dir(command_subclass):

    original_run = command_subclass.run

    def modified_run(self):
        try:
            old_script_dir = self.script_dir  # develop
        except AttributeError:
            old_script_dir = self.install_scripts  # install
        # TODO: A more intelligent way of stitching this together...
        # Warning: script_dir is typically a 'bin' path alongside the
        # lib path, if ever that is somewhere wildly different, this
        # will break.
        # Note: Consider making use of self.prefix, but in some cases
        # that is mislading, e.g. points to /usr when actually
        # everything goes to /usr/local
        new_script_dir = os.path.abspath(
            os.path.join(
                old_script_dir, os.pardir, 'lib', package_name
            )
        )
        log.info("redirecting scripts")
        log.info("  from: {}".format(old_script_dir))
        log.info("    to: {}".format(new_script_dir))
        if hasattr(self, "script_dir"):
            self.script_dir = new_script_dir  # develop
        else:
            self.install_scripts = new_script_dir  # install
        original_run(self)

    command_subclass.run = modified_run
    return command_subclass


@redirect_install_dir
class OverrideDevelop(develop):
    pass


@redirect_install_dir
class OverrideInstall(install):
    pass


setup(
    # cmdclass={
    #     'develop': OverrideDevelop,
    #     'install': OverrideInstall
    # },
    name=package_name,
    version='0.2.3',  # also package.xml
    packages=find_packages(exclude=['tests*', 'docs*']),
    data_files=[('share/' + package_name, ['package.xml'])],
    # scripts=['scripts/py-trees-devel-viewer'], not working, but not critical
    package_data={'py_trees_ros_viewer': ['*.ui', 'html/*', 'images/*']},
    install_requires=[],  # it's all lies (c.f. package.xml, but no use case for this yet)
    extras_require={},
    author='Daniel Stonier',
    maintainer='Daniel Stonier <d.stonier@gmail.com>',
    url='https://github.com/splintered-reality/py_trees_ros_viewer',
    keywords=['ROS', 'ROS2', 'behaviour-trees', 'Qt', 'Visualisation'],
    zip_safe=True,
    classifiers=[
        'Intended Audience :: Developers',
        'License :: OSI Approved :: BSD License',
        'Programming Language :: Python',
        'Topic :: Scientific/Engineering :: Artificial Intelligence',
        'Topic :: Software Development :: Libraries'
    ],
    description=(
        "A Qt-JS hybrid viewer for visualising executing or log-replayed behaviour trees"
    ),
    long_description=(
        "A Qt-JS hybrid viewer for visualising executing or log-replayed behaviour trees"
    ),
    license='BSD',
    # test_suite="tests"
    # tests_require=['nose', 'pytest', 'flake8', 'yanc', 'nose-htmloutput']
    entry_points={
        'console_scripts': [
            'py-trees-tree-viewer = py_trees_ros_viewer.viewer:main',
        ],
    },
)
