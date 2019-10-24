#!/usr/bin/env python

"""
functions used in upload_knowledge ros node
"""


def create_knowledge_unit_dict(update_type, knowledge):
    return {'update_type': update_type, 'knowledge': knowledge}

    
def create_knowledge_dict(
    knowledge_type=0, instance_type='', instance_name='', attribute_name='', function_value=0.0):
    return {
        'knowledge_type': knowledge_type, 'instance_type': instance_type, 'instance_name': instance_name,
        'attribute_name': attribute_name, 'function_value' : function_value
        }


def parse_objects(pddl_objects):
    """
    pddl_objects: list
    first element is ignored, then after a dash is the categories, and all the instances before
    are the objects belonging to that categories.
    
    """
    # ignore first element
    my_objects = pddl_objects[1:]

     # get the categories (after dash)
    indices = [index for index, obj in enumerate(my_objects) if obj == '-']
    categories = [my_objects[index + 1] for index in indices]
    prev_indices = indices[:-1]
    prev_indices.insert(0, 0)

    instances = [
         get_category_elements(my_objects, prev_index, index)
         for prev_index, index in zip(prev_indices, indices)
    ]

    return categories, instances


def get_category_elements(my_objects, start, end):
    # this is a hack since after the second dash we have to ignore two elements,
    # the dash and the subsequent category.
    if start == 0:
        return my_objects[start: end]
    else:
        return my_objects[start + 2: end]