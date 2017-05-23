--------------------------------------------------------------------------------
LIBRARY ieee;
USE ieee.std_logic_1164.ALL;
 
-- Uncomment the following library declaration if using
-- arithmetic functions with Signed or Unsigned values
USE ieee.numeric_std.ALL;
 
ENTITY bensvic2_tb IS
END bensvic2_tb;
 
ARCHITECTURE behavior OF bensvic2_tb IS 
 
    -- Component Declaration for the Unit Under Test (UUT)
 
    COMPONENT bensvic2
    PORT(
         sysclk : IN  std_logic;
         reset_S : IN  std_logic;
         reset_L : IN  std_logic;
         pixelclock_en : IN  std_logic;
         cpuioclock_en : IN  std_logic;
         vsync : OUT  std_logic;
         hsync : OUT  std_logic;
         vgared : OUT  unsigned(3 downto 0);
         vgagreen : OUT  unsigned(3 downto 0);
         vgablue : OUT  unsigned(3 downto 0)
        );
    END COMPONENT;
    

   --Inputs
   signal sysclk : std_logic := '0';
   signal reset_S : std_logic := '0';
   signal reset_L : std_logic := '0';
   signal pixelclock_en : std_logic := '0';
   signal cpuioclock_en : std_logic := '0';

 	--Outputs
   signal vsync : std_logic;
   signal hsync : std_logic;
   signal vgared : unsigned(3 downto 0);
   signal vgagreen : unsigned(3 downto 0);
   signal vgablue : unsigned(3 downto 0);

   -- Clock period definitions
   constant sysclk_period : time := 10 ns;

BEGIN
 
   uut_0: bensvic2 PORT MAP (
	-- Instantiate the Unit Under Test (UUT)
          sysclk => sysclk,
          reset_S => reset_S,
          reset_L => reset_L,
          pixelclock_en => pixelclock_en,
          cpuioclock_en => cpuioclock_en,
          vsync => vsync,
          hsync => hsync,
          vgared => vgared,
          vgagreen => vgagreen,
          vgablue => vgablue
        );

   -- Clock process definitions
   sysclk_process :process
   begin
	sysclk <= '0';
	wait for sysclk_period/2;
	sysclk <= '1';
	wait for sysclk_period/2;
   end process;
 

   -- Stimulus process
   stim_proc: process
   begin		
      -- hold in reset state, then release reset and assert clock_en

      reset_S <= '1';
      reset_L <= '1';
      pixelclock_en <= '0';
      cpuioclock_en <= '0';

      wait for 100 us;

      reset_S <= '0';
      reset_L <= '0';

      -- propogation delay
      wait for 1 ns;

      pixelclock_en <= '1';

      -- insert stimulus here 
      wait;

   end process;

END;
