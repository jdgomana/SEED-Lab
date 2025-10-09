'''
This program determines if "abcd" is contained in the input string using a FSM.
'''

def state0(activity):
    if activity == "a":
        return state1
    else:
        return state0
    
def state1(activity):
    if activity == "a":
        return state1
    elif activity == "b":
        return state2
    else:
        return state0

def state2(activity):
    if activity == "a":
        return state1
    elif activity == "c":
        return state3
    else:
        return state0

def state3(activity):
    if activity == "a":
        return state1
    elif activity == "d":
        return None
    
# create a dictionary to describe the states
state_dictionary = {
    state0 : "start",
    state1 : "a",
    state2 : "b",
    state3 : "c",
}

# Initalization
print("Input a string.")
input = input("Input: ")
state = state0 # initial state as pointer to state0 function
for char in input: 
    new_state = state(char) # launch state machine
    state = new_state # update the next state
    if state == None:
        print("abcd is contained in the string.")
        break
else:
    print("abcd is not contained in the string.")
