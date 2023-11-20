.globl __start

.rodata
    msg0: .string "This is HW1-1: T(n) = 5T(n/2) + 6n + 4, T(1) = 2\n"
    msg1: .string "Enter a number: "
    msg2: .string "The result is: "

.text


__start:
  # Prints msg0
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

################################################################################ 
  # Write your main function here. 
  # Input n is in a0. You should store the result T(n) into t0
  # HW1-1 T(n) = 5T(n/2) + 6n + 4, T(1) = 2, round down the result of division
  # ex. addi t0, a0, 1
    jal x1, f_T
    addi x1, x0, 0
    jal x1, result

f_T:
    addi sp, sp, -16
    sw x1, 8(sp)
    sw a0, 0(sp)
    addi t0, x0, 2
    bge a0, t0, recursive
    addi a0, x0, 2
    addi sp, sp, 16
    jalr x0, 0(x1)
recursive:
    addi t0, x0, 2
    div a0, a0, t0
    jal x1, f_T
    addi t0, x0, 5
    mul t0, a0, t0
    lw a0, 0(sp)
    lw x1, 8(sp)
    addi sp, sp, 16
    addi t1, x0, 6
    mul t1, t1, a0
    add a0, t0, t1
    addi a0, a0, 4
    jalr x0, 0(x1)

################################################################################

result:
    add t0, a0, x0
  # Prints msg2
    addi a0, x0, 4
    la a1, msg2
    ecall

  # Prints the result in t0
    addi a0, x0, 1
    add a1, x0, t0
    ecall
    
  # Ends the program with status code 0
    addi a0, x0, 10
    ecall