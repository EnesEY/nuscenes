import playFile2 

def setup():
  print("My setup")


def my_function():
    setup()
    playFile2.myOtherFunction()
    print("Hello from a function")


my_function()

# user python playFile.py to execute this shit