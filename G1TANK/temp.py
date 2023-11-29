
while True:
    try:
        expectedRows = int(input('Enter number of rows: '))
        if expectedRows <= 0:
            raise ValueError
        break
    except ValueError:
        print("Incorrect value entered")