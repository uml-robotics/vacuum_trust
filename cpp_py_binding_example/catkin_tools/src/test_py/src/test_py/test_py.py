#!/usr/bin/env python


from test_cpp.test_cpp import *
import time

print greet()

initialize()
print "starting counter"
count = 0
while (count < 5):
  print get_count()
  time.sleep(2)
  count = count + 1


pause()
print get_count()
print "pausing count, next numbers should be same"
time.sleep(2)  #waiting at least 1 second to ensure the pausing is applied
print get_count()
time.sleep(2) #next number should be the same
print get_count()
print "disposing"
dispose()
