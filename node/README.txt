Save your node Python scripts in this folder.


# Pseudocode
# Take in image, assume we start on the line
  # We see this by defining cv_image.shape
  # Now how do we use edges & how do we know the size of the image
      # Revise this
# Take one small slice and analyze it for the positions of the two lines
  # Around 20 pixels?
# Makes sure the rightmost distance is equal to the leftmost distance
    # If maximum distance is greater than minimum
    # Make sure we have edge cases, as if it is stuck on a corner (i.e. max = min, or min = 0, or max = 0)
# Move to the right and left with move angular until this is the case
  #I envision this as two for loops
# Then, when they are equal, move forward