import numpy as np


# The origo of the board is located in the middle of tag 0

# Corner coordinates of the markers wrt global metashape frame, with equal orientation as the tag frame
# 84 coordinates describing the four corners of the 21 tags with, IDs from 0 to 20.
# Listed in id-cronological order, starting from the top left corner of each tag, and moving clockwise.
                                #np.array([1.574309, 1.864690, 4.120169], dtype='float32'),#19
                                #np.array([1.578462, 1.861707, 4.340886], dtype='float32'),
                                #np.array([1.579109, 1.640250, 4.338705], dtype='float32'),
                                #np.array([1.576131, 1.642748, 4.117369], dtype='float32'),
t_global_tag_corners_list = [   
                                np.array([1.5791696310043335, 2.089743137359619-2*0.226, 4.4975128173828125], dtype='float32'), #4
                                np.array([1.5834962129592896, 2.0866355895996094-2*0.226, 4.727451324462891], dtype='float32'), #3
                                np.array([1.5853842496871948, 1.8566615581512451-2*0.226, 4.724550247192383], dtype='float32'), #2
                                np.array([1.5810576677322388, 1.8597691059112549-2*0.226, 4.494611740112305], dtype='float32'), #1#49
                                
                                np.array([1.5646326541900635, 2.1111185550689697-2*0.237, 3.7298576831817627], dtype='float32'),#4
                                np.array([1.5689592361450195, 2.10801100730896-2*0.237, 3.959796190261841], dtype='float32'),   #3
                                np.array([1.5708472728729248, 1.8780369758605957-2*0.237, 3.956894636154175], dtype='float32'), #2
                                np.array([1.5665206909179688, 1.8811445236206055-2*0.237, 3.7269561290740967], dtype='float32'),#1 #46
                                
                                np.array([3.556507110595703, 2.0811126232147217-2*0.224, 3.964585542678833], dtype='float32'), #4
                                np.array([3.55344295501709, 2.0809619426727295-2*0.224, 3.7346060276031494], dtype='float32'), #3
                                np.array([3.5532565116882324, 1.8509621620178223-2*0.224, 3.734471082687378], dtype='float32'), #2
                                np.array([3.5563206672668457, 1.8511128425598145-2*0.224, 3.9644505977630615], dtype='float32'), #1 #47

                                np.array([3.566305637359619, 2.105593204498291-2*0.248, 4.69853401184082], dtype='float32'), #4  
                                np.array([3.563241481781006, 2.105442523956299-2*0.248, 4.468554496765137], dtype='float32'), #3 
                                np.array([3.5630550384521484, 1.8754427433013916-2*0.248, 4.468420028686523], dtype='float32'), #2
                                np.array([3.5661191940307617, 1.8755934238433838-2*0.248, 4.698399543762207], dtype='float32'), #1 #45              
                                ]


# Origin of the tag frame expressed in the global metashape frame
origin_global_tag = np.array([2.1207, 1.9766, 4.7421], dtype = 'float32')


# Elementwise subtracion of the origin vector from the tag corners
t_tag_corners_list = [vector - origin_global_tag for vector in t_global_tag_corners_list]


# Initialize the list
pos_board_4x4 = []

# Loop through the t_tag_corners_list in steps of 4 up to the 84th element
for i in range(0, 16, 4):  # from 0 to 80 by steps of 4 (to include up to t_tag_corners_list[83])
    # Create an array of four tuples of coordinates and append to pos_board
    pos_board_4x4.append(np.array([(t_tag_corners_list[i][0], t_tag_corners_list[i][1], t_tag_corners_list[i][2]), (t_tag_corners_list[i+1][0], t_tag_corners_list[i+1][1], t_tag_corners_list[i+1][2]), (t_tag_corners_list[i+2][0], t_tag_corners_list[i+2][1], t_tag_corners_list[i+2][2]), (t_tag_corners_list[i+3][0], t_tag_corners_list[i+3][1], t_tag_corners_list[i+3][2])], dtype='float32'))

# Now pos_board contains groups of 4 coordinates, from t_tag_corners_list[0] to t_tag_corners_list[83]

# print('Jadda')

#for formatted_array in pos_board_4x4:
#    print(formatted_array)

# print(pos_board[20])

# Defines the Ids of the markers in the board
id_board_4x4 = np.array([[49], [46], [47], [45]])