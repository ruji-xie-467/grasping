import math


a = 0

b = 1
while (1):
    try:
        a = input("~~~")
        c = b/a
    except Exception as e:
        print(e)
    else:
        print('else')
        break