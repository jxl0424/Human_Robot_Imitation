# based on example URscript provided by onrobot to operate
# RG grippers

class RG_Message_Generator:
	def __init__(self, gripper_type):
		assert gripper_type == "RG6" or gripper_type == "RG2", """gripper type must be "RG2" or "RG6" """
		# set appropriate values for each gripper
		# data from examples provided by onrobot
		if gripper_type == "RG6":
			self.multiplier = 161
			self.divider = 5
		elif gripper_type == "RG2":
			self.multiplier = 111
			self.divider = 2

	def message(self, width, force):
		# generate string containing URScript functions to 
		# move gripper to position and call with desired values
		message = """def RG():
  def bit(input):
    msb=65536
    local i=0
    local output=0
    while i<17:
      set_digital_out(8,True)
      if input>=msb:
        input=input-msb
        set_digital_out(9,False)
      else:
        set_digital_out(9,True)
      end
      if get_digital_in(8):
        out=1
      end
      sync()
      set_digital_out(8,False)
      sync()
      input=input*2
      output=output*2
      i=i+1
    end
    return output
  end
  def RG2(target_width=110, target_force=40):
    bit(0)
    sleep(0.024)
    rg_data=floor(target_width)*4
    rg_data=rg_data+floor(target_force/{})*4*{}
    rg_data=rg_data+32768
    bit(rg_data)
    while get_digital_in(9) == True:
      sync()
    end
    while get_digital_in(9) == False:
      sync()
    end
  end
  RG2({}, {})
end
""".format(self.divider, self.multiplier, width, force)
		return message
