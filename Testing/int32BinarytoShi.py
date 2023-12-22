def toBinary(n):
    return (toBinary(n // 2) if n > 1 else '') + str(n % 2)
 
 
if __name__ == '__main__':
 
    n = 20
    print(f'The binary representation of {n} is', toBinary(n))