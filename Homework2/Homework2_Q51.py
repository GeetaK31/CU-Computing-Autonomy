#Question: Define a class named American and its subclass NewYorker.

class American:
    nationality = "American"

class NewYorker(American):
    city = "New York"

new_yorker = NewYorker()

print("Nationality:", new_yorker.nationality)
print("City:", new_yorker.city)

