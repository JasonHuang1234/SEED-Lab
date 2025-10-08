#state machine to determine if the string abcd is in the function
#Initial state, detecing a  it moves next
def state0(stringy):
    if stringy != 'a':
        return state0
    elif stringy == "a":
        return stateA
#At A it moves to stateb if it gets inputted b otherwise return to 0
def stateA(stringy):
    if stringy == "a":
        return state0
    elif stringy == 'b':
        return stateB
    return state0
#Same thing as before
def stateB(stringy):
    if stringy == "c":
        return stateC
    elif stringy == 'a':
        return stateA
    return state0

def stateC(stringy):
    if stringy == "d":
        return stateD
    elif stringy == 'a':
        return stateA
    return state0
#Just returns state D to allow the loop to stop and go print
def stateD(stringy):
    return stateD

#State dictionary
state_dictionary = {
    state0 : "No A",
    stateA : "Found A",
    stateB : "Found B",
    stateC : "Found C",
    stateD : "Found D"
}

state = state0
#variable to determine if abcd was found
found = False
user_input = input("Please enter something: ")
for i in user_input:
    print(state)
    #Ensures states are correct and runs through state machine
    if state != None:
        new_state = state(i)
        state = new_state
    elif state == None:
        continue
    #Breaks the loop to go print
    if state == stateD:
        found = True
        break

#prints if it found abcd
if found:
    print("abcd")
else:
    print("Didn't find")

