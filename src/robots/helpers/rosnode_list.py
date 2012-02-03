#! /usr/bin/env python
import os

def rosnode_list(x):
        
    node_list = os.popen("rosnode list | grep " + str(x))
    list = []
    for node in node_list.readlines():
        #print node
        print (node.replace("/", ""))
        
        list.append(str(node.replace("/", "")))
        
    return list
    
if __name__=="__main__":
    print("Which node ?")
    x = raw_input()
              

    list = rosnode_list(x)
        
    print("#############")
    print(list)
    
    for node in list:
        #print node, type(node)
        if node == "calibrate_pr2\n":
            print("OK")
        else:
            print("Echec")

        

