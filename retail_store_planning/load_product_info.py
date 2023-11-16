#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from rosplan_knowledge_msgs.srv import KnowledgeUpdateServiceArray
from diagnostic_msgs.msg import KeyValue
from rosplan_knowledge_msgs.msg import KnowledgeItem

def formulate_new_goal(object):
	update_type = [1]
	knowledge = assign_keyvalues(object)

	rospy.wait_for_service('/rosplan_knowledge_base/update_array')
	try:
		function = rospy.ServiceProxy('/rosplan_knowledge_base/update_array', KnowledgeUpdateServiceArray)
		result = function(update_type, knowledge)

	except rospy.ServiceException as e:
		result = 0
		print("Service call failed: %s"%e)

	return result

def assign_keyvalues(object):
	my_knowledge = []
	my_knowledge.append(KnowledgeItem())
	my_knowledge[0].knowledge_type = 1
	my_knowledge[0].attribute_name = "stored"
	keyvalue1 = KeyValue()
	keyvalue1.key = "obj"
	keyvalue1.value = object

	my_knowledge[0].values.append(keyvalue1)

	return my_knowledge

def formulate_new_instance(object):
	update_type = [0]
	knowledge = assign_keyvalues_instance(object)

	rospy.wait_for_service('/rosplan_knowledge_base/update_array')
	try:
		function = rospy.ServiceProxy('/rosplan_knowledge_base/update_array', KnowledgeUpdateServiceArray)
		result = function(update_type, knowledge)

	except rospy.ServiceException as e:
		result = 0
		print("Service call failed: %s"%e)

	return result

def assign_keyvalues_instance(object):
	my_knowledge = []
	my_knowledge.append(KnowledgeItem())
	my_knowledge[0].knowledge_type = 0
	my_knowledge[0].instance_type = "object"
	my_knowledge[0].instance_name = object

	return my_knowledge

def stock_object(object):
	update_type = [0]
	knowledge = object_at(object)

	rospy.wait_for_service('/rosplan_knowledge_base/update_array')
	try:
		function = rospy.ServiceProxy('/rosplan_knowledge_base/update_array', KnowledgeUpdateServiceArray)
		result = function(update_type, knowledge)

	except rospy.ServiceException as e:
		result = 0
		print("Service call failed: %s"%e)

	return result

def object_at(object):
	my_knowledge = []
	my_knowledge.append(KnowledgeItem())
	my_knowledge[0].knowledge_type = 1
	my_knowledge[0].attribute_name = "object-at"
	keyvalue1 = KeyValue()
	keyvalue1.key = "obj"
	keyvalue1.value = object
	keyvalue2 = KeyValue()
	keyvalue2.key = "wp"
	keyvalue2.value = "wp_table_1"
	my_knowledge[0].values.append(keyvalue1)
	my_knowledge[0].values.append(keyvalue2)
	return my_knowledge

def add_product_info(object, predicate):
	update_type = [0]
	knowledge = object_predicate(object, predicate)
    
	rospy.wait_for_service('/rosplan_knowledge_base/update_array')
	try:
		function = rospy.ServiceProxy('/rosplan_knowledge_base/update_array', KnowledgeUpdateServiceArray)
		result = function(update_type, knowledge)

	except rospy.ServiceException as e:
		result = 0
		print("Service call failed: %s"%e)

	return result

def object_predicate(object, predicate):
	my_knowledge = []

	my_knowledge.append(KnowledgeItem())
	my_knowledge[0].knowledge_type = 1
	my_knowledge[0].attribute_name = predicate
	keyvalue1 = KeyValue()
	keyvalue1.key = "obj"
	keyvalue1.value = object
	my_knowledge[0].values.append(keyvalue1)
	return my_knowledge



if __name__ == '__main__':
    product_list = []

    product1 = ["april_tag_cube_8",  "hagelslag", 0.5]
    product2 = ["april_tag_cube_23", "hagelslag", 0.1]
    product3 = ["april_tag_cube_15", "yoghurt", 1.0]
    product4 = ["april_tag_cube_24", "yoghurt", 0.5]

    product_list.append(product1)
    product_list.append(product2)
    product_list.append(product3)
    product_list.append(product4)

    weight_full_hag = 0.5
    weight_full_yog = 1.0

    for product in product_list:
        #INSERT CODE TO CREATE INSTANCE
        formulate_new_instance(product[0])
        formulate_new_goal(product[0])
        print(product[0])
        stock_object(product[0])
        
        if product[1] == "hagelslag":
            add_product_info(product[0], "is-not-refrigerated")
            if product[2] >= weight_full_hag:
                add_product_info(product[0], "is-full")
                print("full")
            elif product[2] < weight_full_hag:
                add_product_info(product[0], "is-not-full")
                print("not full")
            else:
                  print("weight error")

        elif product[1] == "yoghurt":
            add_product_info(product[0], "is-refrigerated")
            if product[2] >= weight_full_yog:
                add_product_info(product[0], "is-full")
                print("full")
            elif product[2] < weight_full_yog:
                add_product_info(product[0], "is-not-full")
                print("not full")
            else:
                print("weight error")
        
        else:
            print("Unknown product. please add", product[1], "to the ontology")
