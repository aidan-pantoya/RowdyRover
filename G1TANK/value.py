while True:
    try:
        expectedRows = int(input('How many rows? (as an integer)'))
        if expectedRows <= 0:
            raise ValueError
        break
    except ValueError:
        print("Incorrect value entered")