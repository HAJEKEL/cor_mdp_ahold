from datetime import datetime, timedelta

# Store the current date in yyyy-mm-dd format
current_date = datetime.today().strftime('%Y-%m-%d')

# Function to increment the current date by one day
def increment_date():
    global current_date
    date = datetime.strptime(current_date, '%Y-%m-%d')
    new_date = date + timedelta(days=1)
    current_date = new_date.strftime('%Y-%m-%d')

for i in range(30):
    increment_date()
    print(current_date)
