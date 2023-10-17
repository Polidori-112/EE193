def convert_to_comma_separated(input_filename, output_filename):
    with open(input_filename, 'r') as infile:
        numbers = infile.readlines()
    
    # Remove newline characters and filter out any empty lines
    numbers = [num.strip() for num in numbers if num.strip()]

    # Join the numbers with commas
    comma_separated = ',\n'.join(numbers)
    
    with open(output_filename, 'w') as outfile:
        outfile.write(comma_separated)

# Example usage:
convert_to_comma_separated('Lab6/include/data2.txt', 'Lab6/include/data2_wcommas.txt')
