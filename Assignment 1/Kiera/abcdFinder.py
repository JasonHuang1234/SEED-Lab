# sets state to starting position and prompts the user for a string
state = 0
userString = input("Please enter a string")

# determines if the next letter is a
def start_state(letter):
    if letter == "a":
        return 1
    else:
        return 0

# previous letter should be a, determines if next is a or b, otherwise restarts
def state_a(letter):
    if letter == "b":
        return 2
    elif letter == "a":
        return 1
    else:
        return 0

# previous letter should be b, determines if next is a or c, otherwise restarts
def state_b(letter):
    if letter == "c":
        return 3
    elif letter == "a":
        return 1
    else:
        return 0

# previous letter should be c, determines if next is a or d, otherwise restarts
# if the letter found is d the string contains 'abcd'
def state_c(letter):
    if letter == "d":
        print("The string you provided contains 'abcd'")
        return 0
    elif letter == "a":
        return 1
    else:
        return 0

# create a dictionary to describe the states
state_dictionary = {
    0 : start_state,
    1 : state_a,
    2 : state_b,
    3 : state_c,
}

i = 0
# checks every letter in the user provided string and uses state machine dictionanry
# to determine if 'abcd' is found anywhere in it
for letter in userString:
    state = state_dictionary[state](letter)
print("Done with state machine")