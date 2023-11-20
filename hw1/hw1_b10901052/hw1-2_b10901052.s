.globl __start

.rodata
    msg0: .string "This is HW1-2: \n"
    msg1: .string "Enter shift: "
    msg2: .string "Plaintext: "
    msg3: .string "Ciphertext: "
.text

################################################################################
  # print_char function
  # Usage: 
  #     1. Store the beginning address in x20
  #     2. Use "j print_char"
  #     The function will print the string stored from x20 
  #     When finish, the whole program with return value 0

print_char:
    addi a0, x0, 4
    la a1, msg3
    ecall
  
    add a1,x0,x20
    ecall

  # Ends the program with status code 0
    addi a0,x0,10
    ecall
    
################################################################################

__start:
  # Prints msg
    addi a0, x0, 4
    la a1, msg0
    ecall

  # Prints msg1
    addi a0, x0, 4
    la a1, msg1
    ecall
  # Reads an int
    addi a0, x0, 5
    ecall
    add a6, a0, x0
    
  # Prints msg2
    addi a0, x0, 4
    la a1, msg2
    ecall
    
    addi a0,x0,8
    li a1, 0x10150
    addi a2,x0,2047
    ecall
  # Load address of the input string into a0
    add a0,x0,a1


################################################################################ 
  # Write your main function here. 
  # a0 stores the begining Plaintext
  # x16 stores the shift
  # Do store 66048(0x10200) into x20 
  # ex. j print_char

encryption:
    li x20, 0x10200 # save encryption text
    addi x21, x0, 48 # record " " number
    addi sp, sp, -8
    sw x19, 0(sp)
    add x19, x0, x0 # i = 0
L1:
    add t0, x19, a0 # addr of plaintext
    lbu t1, 0(t0)     # t1 = pt[i]
    addi t2, x0, 10   # t2 store value for comparison
    beq t1, t2, L2    # if pt[i] == 10, exit
    addi t2, x0, 32
    beq t1, t2, space
    add t1, t1, x16   # shift
    addi t2, x0, 97
    blt t1, t2, under
    addi t2, x0, 122
    bgt t1, t2, over
    jal x0, next
space:
    add t1, x21, x0
    addi x21, x21, 1
    jal x0, next
under:
    addi t1, t1, 26
    jal x0, next
over:
    addi t1, t1, -26
    jal x0, next
next:
    add t0, x19, x20
    sb t1, 0(t0)
    addi x19, x19, 1
    jal x0, L1

L2:
    lw x19, 0(sp)
    addi sp, sp, 8
    j print_char
################################################################################

