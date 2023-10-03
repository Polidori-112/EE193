import matplotlib.pyplot as plt

def read_data(filename):
    """
    Read numbers from a file where each line contains one number.
    
    Args:
    - filename (str): Name of the file to read from.

    Returns:
    - list: A list of numbers from the file.
    """
    with open(filename, 'r') as f:
        numbers = [float(line.strip()) for line in f]
    return numbers

def plot_data(numbers):
    """
    Plot numbers over time.

    Args:
    - numbers (list): A list of numbers to plot.
    """
    plt.plot(numbers)
    plt.xlabel('Time')
    plt.ylabel('Value')
    plt.title('Numbers over Time')
    plt.savefig('output2.png', format='png')
    plt.show()

if __name__ == "__main__":
    data_filename = "data2.txt"
    numbers = read_data(data_filename)
    plot_data(numbers)
