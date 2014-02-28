#!/usr/bin/env python

"""
Given: 
PTT<S2,(M20,F20_20_B,F20_20_G,R20,V20),S1>

Return:
BTT<initialsituation(<S2,(M20,F20_20_B,F20_20_G,R20,V20)>);goalsituation(<S1,line(M20,F20_20_B,F20_20_G,R20,V20)>)>
BTT<initialsituation(<S2,(M20,F20_20_B,F20_20_G,R20,V20)>);goalsituation(<S1,line(M20,F20_20_B,F20_20_G,R20,V20)>)>

HACK required for WC2013

"""

import re

def main():
    teststring = "PTT<S2,(M20,F20_20_B,F20_20_G,R20,V20),S1>"
    return ppt2btt(teststring)
    
def ppt2btt(ptt_string):
    objects = re.findall("\(.*?\)",ptt_string)
    objects = objects[0] 
    #print objects

    source = re.findall("<.*?,",ptt_string)[0]
    #print source

    destination = re.findall(".?.?>",ptt_string)[0]
    destination = re.match(".\w",destination).group()
    #print destination
    #destination = destination.group()

    result = "BTT<initialsituation(" + source
    result = result + objects + ">);"
    result = result + "goalsituation(<" + destination + ","
    result = result + "line" + objects + ">)>"

    return result
if __name__ == '__main__':
    r = main()
    print r