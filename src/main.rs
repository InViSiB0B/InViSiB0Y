use std::time::Instant;
mod cpu;
mod mmu;
use cpu::Cpu;
use mmu::MMU;

fn main() 
{
    // Initialize CPU and memory
    let mut cpu = Cpu::new();

    // Load the ROM
    let rom_data = std::fs::read("C:\\Projects\\InViSiB0Y\\gb-test-roms-master\\cpu_instrs\\cpu_instrs.gb")
        .expect("Failed to open test ROM");
    
    // Create the MMU with the new ROM data
    let mut mmu = MMU::new(rom_data);
    
    println!("Starting CPU execution - running until completion...");

    // Set up a buffer for collecting serial output
    let mut serial_output = String::new();
    
    // Initialize performance tracking
    let start_time = Instant::now();
    let mut instructions_executed: u64 = 0;

    // Run the emulation
    loop 
    {
        // Execute one CPU instruction
        if !cpu.step(&mut mmu)
        {
            println!("CPU halted or error detected");
            break
        }

        instructions_executed = instructions_executed.wrapping_add(1);

        if instructions_executed % 1_000_000 == 0 
        {
            println!("Executed {} million instructions, time elapsed: {:?}",
                        instructions_executed / 1_000_000, start_time.elapsed());
        }

        // Check for serial output
        if let Some(output_char) = mmu.check_serial_output()
        {
            print!("{}", output_char);
            serial_output.push(output_char);
        }
    }
    
    // After the loop, dump CPU state and results
    println!("\nExecution completed in {:?} with {} instructions executed", 
             start_time.elapsed(), instructions_executed);
    
    println!("Final CPU state: PC={:#06x}, SP={:#06x}", cpu.pc, cpu.sp);
    println!("Registers: A={:#04x} F={:#04x} BC={:#04x} DE={:#04x} HL={:#04x}",
        cpu.a, cpu.f,
        ((cpu.b as u16) << 8) | cpu.c as u16,
        ((cpu.d as u16) << 8) | cpu.e as u16,
        ((cpu.h as u16) << 8) | cpu.l as u16);
    
    if serial_output.is_empty() {println!("No serial output detected from the test ROM.");} 
    else {println!("Complete serial output: {}", serial_output);}
}