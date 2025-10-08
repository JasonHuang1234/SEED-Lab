import numpy as np


#Takes in a lst input
def counter(lst):
    #variables initialized
    numbers = []
    numbertrack = []
    appearences = []
    lster = []
    index = 0
    #Runs through the list, 
    for i in lst:
        if i in numbers:
            #this code upticks a counter for numbers already in a array
            index = numbers.index(i)
            appearences[index] = appearences[index] + 1
        else:
            #This code adds every number to a array and starts counting
            numbers.append(i)
            index = numbers.index(i)
            appearences.append(0)
            appearences[index] = appearences[index] + 1
    #This finds the value of the maximum counts
    most = max(appearences)
    #This code creates a new array of the numbers corresponding to the max counts
    for i in range(len(appearences)):
        if appearences[i] == most:
            numbertrack.append(numbers[i])
    #This creates a returnable array of the numbers counted as well as counts for it.
    lster.append(numbertrack)
    lster.append(most)


    return lster

maximum = 0
minimum = 0
num38in = 0
numbersrepeatedthemost = []
amountofrepeats = []
Sorted_list = []
even_numbers = []
#This opens the data file
with open('datafile.txt' , 'r') as f:
    b = eval(f.read())
#Finds some basic stuff
maximum = max(b)
minimum = min(b)
numpified = np.array(b)
sorty = np.sort(numpified)


#Parses the inputted file, and finds the location of 38
for i in range(len(b)):
    if b[i] == 38:
        num38in = i
        break
#Runs the counter function to get a list of maximum numbers
listy = counter(b)
even = []
#This finds the even numbers 
for i in sorty:
    if i%2 == 0:
        even.append(i)
#This initializes the even array and sorts it
even = np.array(even)
even = np.sort(even)



#prints outputs
print(f"The Numbers with the most appearences are {listy[0]} and they appear {listy[1]} times.")
print(maximum)
print(minimum)
print(f"38 appears at index {num38in}")
print(f"The sorted array is {sorty}.")
print(f"The even numbers are {even}.")





