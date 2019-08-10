#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# License: BSD
#   https://github.com/splintered-reality/py_trees_ros_viewer/raw/devel/LICENSE
#
##############################################################################
# Documentation
##############################################################################
"""
Demo trees to feed to the web app
"""
##############################################################################
# Imports
##############################################################################

import copy

##############################################################################
# Methods
##############################################################################


def create_demo_tree_definition():
    tree = {
        'timestamp': 1563938995,
        'visited_path': ['1', '2', '7'],
        'behaviours': {
            '1': {
                'id': '1',
                'status': 'RUNNING',
                'name': 'Selector',
                'colour': '#00FFFF',
                'children': ['2', '3', '4', '6'],
                'data': {
                    'Type': 'py_trees.composites.Selector',
                    'Feedback': "Decision maker",
                },
            },
            '2': {
                'id': '2',
                'status': 'RUNNING',
                'name': 'Sequence',
                'colour': '#FFA500',
                'children': ['7', '8', '9'],
                'data': {
                    'Type': 'py_trees.composites.Sequence',
                    'Feedback': "Worker"
                },
            },
            '3': {
                'id': '3',
                'status': 'INVALID',
                'name': 'Parallel',
                'details': 'SuccessOnOne',
                'colour': '#FFFF00',
                'children': ['10', '11'],
                'data': {
                    'Type': 'py_trees.composites.Parallel',
                    'Feedback': 'Baked beans is good for your heart, baked beans makes you',
                },
            },
            '4': {
                'id': '4',
                'status': 'RUNNING',
                'name': '&#x302; &#x302; Decorator',
                'colour': '#DDDDDD',
                'children': ['5'],
                'data': {
                    'Type': 'py_trees.composites.Decorator',
                    'Feedback': 'Wearing the hats',
                },
            },
            '5': {
                'id': '5',
                'status': 'INVALID',
                'name': 'Decorated Beyond The Beliefs of an Agnostic Rhino',
                'colour': '#555555',
                'data': {
                    'Type': 'py_trees.composites.Behaviour',
                    'Feedback': "...."
                },
            },
            '6': {
                'id': '6',
                'status': 'INVALID',
                'name': 'Behaviour',
                'colour': '#555555',
                'data': {
                    'Type': 'py_trees.composites.Behaviour',
                    'Feedback': "..."
                },
            },
            '7': {
                'id': '7',
                'status': 'RUNNING',
                'name': 'Worker A',
                'colour': '#555555',
                'data': {
                    'Type': 'py_trees.composites.Behaviour',
                    'Feedback': "..."
                },
            },
            '8': {
                'id': '8',
                'status': 'INVALID',
                'name': 'Worker B',
                'colour': '#555555',
                'data': {
                    'Type': 'py_trees.composites.Behaviour',
                    'Feedback': "..."
                },
            },
            '9': {
                'id': '9',
                'status': 'INVALID',
                'name': 'Worker C',
                'colour': '#555555',
                'data': {
                    'Type': 'py_trees.composites.Behaviour',
                    'Feedback': "..."
                },
            },
            '10': {
                'id': '10',
                'status': 'INVALID',
                'name': 'Foo',
                'colour': '#555555',
                'data': {
                    'Type': 'py_trees.composites.Behaviour',
                    'Feedback': "..."
                },
            },
            '11': {
                'id': '11',
                'status': 'INVALID',
                'name': 'Bar',
                'colour': '#555555',
                'data': {
                    'Type': 'py_trees.composites.Behaviour',
                    'Feedback': "..."
                },
            },
        }
    }
    return tree


def create_demo_tree_list():
    trees = []
    tree = create_demo_tree_definition()
    trees.append(copy.deepcopy(tree))
    tree['visited_path'] = ['1', '2', '7', '8']
    tree['behaviours']['7']['status'] = 'SUCCESS'
    tree['behaviours']['8']['status'] = 'RUNNING'
    trees.append(copy.deepcopy(tree))
    # sequence
    tree['visited_path'] = ['1', '2', '3', '4', '5', '8', '9', '10', '11']
    tree['behaviours']['2']['status'] = 'FAILURE'
    tree['behaviours']['8']['status'] = 'SUCCESS'
    tree['behaviours']['9']['status'] = 'FAILURE'
    # parallel
    tree['behaviours']['3']['status'] = 'SUCCESS'
    tree['behaviours']['10']['status'] = 'SUCCESS'
    tree['behaviours']['11']['status'] = 'RUNNING'
    # decorated
    tree['behaviours']['4']['status'] = 'RUNNING'
    tree['behaviours']['5']['status'] = 'RUNNING'
    trees.append(copy.deepcopy(tree))
    return trees
