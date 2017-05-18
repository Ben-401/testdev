--
-- Written by
--    Paul Gardner-Stephen <hld@c64.org>  2013-2014
--
-- *  This program is free software; you can redistribute it and/or modify
-- *  it under the terms of the GNU Lesser General Public License as
-- *  published by the Free Software Foundation; either version 3 of the
-- *  License, or (at your option) any later version.
-- *
-- *  This program is distributed in the hope that it will be useful,
-- *  but WITHOUT ANY WARRANTY; without even the implied warranty of
-- *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
-- *  GNU General Public License for more details.
-- *
-- *  You should have received a copy of the GNU Lesser General Public License
-- *  along with this program; if not, write to the Free Software
-- *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA
-- *  02111-1307  USA.
----------------------------------------------------------------------------------
-- Company: 
-- Engineer: 
-- 
-- Create Date:    19:11:30 01/02/2014 
-- Design Name: 
-- Module Name:    vga - Behavioral 
-- Project Name: 
-- Target Devices: 
-- Tool versions: 
-- Description: 
--
-- Dependencies: 
--
-- Revision: 
-- Revision 0.01 - File Created
-- Additional Comments: 
--
----------------------------------------------------------------------------------
library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use ieee.numeric_std.all;
use Std.TextIO.all;
use work.debugtools.all;

-- Uncomment the following library declaration if using
-- arithmetic functions with Signed or Unsigned values
--use IEEE.NUMERIC_STD.ALL;

-- Uncomment the following library declaration if instantiating
-- any Xilinx primitives in this code.
--library UNISIM;
--use UNISIM.VComponents.all;

entity bensvic is
  Port (
    sysclk        : in std_logic;
	 reset2        : in std_logic;
	 pixelclock_en : in std_logic;
	 cpuioclock_en : in std_logic;
    

    
    ----------------------------------------------------------------------
    -- VGA output
    ----------------------------------------------------------------------
    vsync : out  STD_LOGIC;
    hsync : out  STD_LOGIC;
    vgared : out  UNSIGNED (3 downto 0);
    vgagreen : out  UNSIGNED (3 downto 0);
    vgablue : out  UNSIGNED (3 downto 0)
	 
	 
    );
end bensvic;

architecture Behavioral of bensvic is

  constant  h_pulse  :  INTEGER   := 208;   --horiztonal sync pulse width in pixels
  constant  h_bp     :  INTEGER   := 336;   --horiztonal back porch width in pixels
  constant  h_pixels :  INTEGER   := 1920;  --horiztonal display width in pixels
  constant  h_fp     :  INTEGER   := 128;   --horiztonal front porch width in pixels
  constant  h_pol    :  STD_LOGIC := '0';   --horizontal sync pulse polarity (1 = positive, 0 = negative)

  constant  v_pulse  :  INTEGER   := 3;     --vertical sync pulse width in rows
  constant  v_bp     :  INTEGER   := 38;    --vertical back porch width in rows
  constant  v_pixels :  INTEGER   := 1200;  --vertical display width in rows
  constant  v_fp     :  INTEGER   := 1;     --vertical front porch width in rows
  constant  v_pol    :  STD_LOGIC := '1';  --vertical sync pulse polarity (1 = positive, 0 = negative)

  CONSTANT  h_period  :  INTEGER := h_pulse + h_bp + h_pixels + h_fp;  --total number of pixel clocks in a row
  CONSTANT  v_period  :  INTEGER := v_pulse + v_bp + v_pixels + v_fp;  --total number of rows in column
  
begin


  PROCESS(sysclk, reset2)

    VARIABLE h_count  :  INTEGER RANGE 0 TO h_period - 1 := 0;  --horizontal counter (counts the columns)
    VARIABLE v_count  :  INTEGER RANGE 0 TO v_period - 1 := 0;  --vertical counter (counts the rows)

  BEGIN
  
    IF(reset2 = '0') THEN  --reset asserted
      h_count := 0;         --reset horizontal counter
      v_count := 0;         --reset vertical counter
      hsync <= NOT h_pol;  --deassert horizontal sync
      vsync <= NOT v_pol;  --deassert vertical sync
      
    ELSIF rising_edge(sysclk) THEN
	 if (pixelclock_en = '1') then

      --counters
      IF(h_count < h_period - 1) THEN    --horizontal counter (pixels)
        h_count := h_count + 1;
      ELSE
        h_count := 0;
        IF(v_count < v_period - 1) THEN  --veritcal counter (rows)
          v_count := v_count + 1;
        ELSE
          v_count := 0;
        END IF;
      END IF;

      --horizontal sync signal
      IF(h_count < h_pixels + h_fp OR h_count > h_pixels + h_fp + h_pulse) THEN
        hsync <= NOT h_pol;    --deassert horiztonal sync pulse
      ELSE
        hsync <= h_pol;        --assert horiztonal sync pulse
      END IF;
      
      --vertical sync signal
      IF(v_count < v_pixels + v_fp OR v_count > v_pixels + v_fp + v_pulse) THEN
        vsync <= NOT v_pol;    --deassert vertical sync pulse
      ELSE
        vsync <= v_pol;        --assert vertical sync pulse
      END IF;
      
      --set pixel coordinates
      IF(h_count < h_pixels) THEN  --horiztonal display time
		  if (h_count < 10) or
		    ((h_pixels - h_count) < 10) or
			 (v_count < 10) or
		    ((v_pixels - v_count) < 10)
		  then
          vgared <= to_unsigned(15,4);
          vgagreen <= to_unsigned(15,4);
          vgablue <= to_unsigned(15,4);
		  else
          vgared   <= to_unsigned(h_count/32,4);         --set horiztonal pixel coordinate
          vgagreen <= to_unsigned(v_count/16,4);         --set horiztonal pixel coordinate
          vgablue  <= to_unsigned(v_count/8,4);         --set horiztonal pixel coordinate
		  end if;
		else
        vgared <= to_unsigned(0,4);
        vgagreen <= to_unsigned(0,4);
        vgablue <= to_unsigned(0,4);
		end if;
		

    END IF;
    END IF;
  END PROCESS;

end Behavioral;

