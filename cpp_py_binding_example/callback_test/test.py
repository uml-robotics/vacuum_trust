import callback_test

def add_func(a, b):
  print a + b


callback_test.set_callback(add_func)
callback_test.call_f(1, 3)
