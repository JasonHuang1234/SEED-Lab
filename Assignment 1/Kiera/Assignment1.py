#loads data
with open('datafile.txt','r') as f:
    b = eval(f.read())

#prints min and max in data set
print("The highest number found was %d" % max(b))
print("The lowest number found was %d" % min(b))

#prints the index that 38 is found at
print("The first index that 38 was found was %d" % b.index(38))

from collections import Counter #helpful import for array actions

#finds the sorted list of number frequency and outputs the details of the most common
common = Counter(b).most_common()
print("The most repeated number is %d. It is found %d time(s) in this data set." % (common[0][0], common[1][0]))

#imports and uses numpy to sort list
import numpy
print("Here is the dataset listed in asending order")
print(numpy.sort(numpy.array(b)))

#prints a sorted list of the even numbers in list b
print("Here are all the even numbers found in the list")
print(sorted(x for x in b if x % 2 == 0))