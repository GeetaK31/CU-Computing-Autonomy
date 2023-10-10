#Question: Write a function to compute 5/0 and use try/except to catch the exceptions.
def division():
    try:
        result = 5 / 0
        return result
    except ZeroDivisionError as e:
        print("Error:", e)
        return None

# Call the function to compute division and catch exceptions
div_result = division()

# Clear workspace variables
del div_result
