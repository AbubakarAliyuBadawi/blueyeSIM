import numpy as np


# The origo of the board is located in the middle of tag 0

# Corner coordinates of the markers wrt global metashape frame, with equal orientation as the tag frame
# 84 coordinates describing the four corners of the 21 tags with, IDs from 0 to 20.
# Listed in id-cronological order, starting from the top left corner of each tag, and moving clockwise.
t_global_tag_corners_list = [   np.array([2.045516, 2.052526, 4.743317], dtype='float32'),#0
                                np.array([2.196485, 2.052021, 4.741942], dtype='float32'),
                                np.array([2.195989, 1.900467, 4.740871], dtype='float32'),
                                np.array([2.044927, 1.901252, 4.742156], dtype='float32'),
                                np.array([1.681250, 1.550309, 3.732092], dtype='float32'),#1
                                np.array([1.832481, 1.552090, 3.730770], dtype='float32'),
                                np.array([1.834302, 1.401129, 3.729769], dtype='float32'),
                                np.array([1.683171, 1.398995, 3.731514], dtype='float32'),
                                np.array([2.441016, 1.551369, 3.723809], dtype='float32'),#2
                                np.array([2.592715, 1.550104, 3.722663], dtype='float32'),
                                np.array([2.591342, 1.398833, 3.723274], dtype='float32'),
                                np.array([2.440099, 1.400068, 3.723423], dtype='float32'),
                                np.array([2.008570, 1.103020, 4.737178], dtype='float32'),#3
                                np.array([2.230176, 1.102463, 4.736394], dtype='float32'),
                                np.array([2.229312, 0.880296, 4.735617], dtype='float32'),
                                np.array([2.007510, 0.880850, 4.736354], dtype='float32'),
                                np.array([2.056640, 1.554347, 3.726721], dtype='float32'),#4
                                np.array([2.207685, 1.554170, 3.725683], dtype='float32'),
                                np.array([2.207369, 1.403005, 3.725315], dtype='float32'),
                                np.array([2.056384, 1.403304, 3.726506], dtype='float32'),
                                np.array([2.684994, 1.251511, 4.731256], dtype='float32'),#5
                                np.array([2.791189, 1.251470, 4.730640], dtype='float32'),
                                np.array([2.791446, 1.145207, 4.731605], dtype='float32'),
                                np.array([2.685161, 1.145314, 4.731905], dtype='float32'),
                                np.array([2.077524, 1.300923, 3.724992], dtype='float32'),#6
                                np.array([2.184048, 1.301713, 3.725385], dtype='float32'),
                                np.array([2.184621, 1.195112, 3.725649], dtype='float32'),
                                np.array([2.077986, 1.194811, 3.726449], dtype='float32'),
                                np.array([1.841074, 1.947204, 4.736925], dtype='float32'),#7
                                np.array([1.911646, 1.948187, 4.736664], dtype='float32'),
                                np.array([1.912691, 1.877652, 4.736209], dtype='float32'),
                                np.array([1.842057, 1.876600, 4.736302], dtype='float32'),
                                np.array([1.606422, 1.815495, 4.736138], dtype='float32'),#8
                                np.array([1.676400, 1.814957, 4.738200], dtype='float32'),
                                np.array([1.676174, 1.744618, 4.738680], dtype='float32'),
                                np.array([1.606246, 1.745029, 4.735842], dtype='float32'),
                                np.array([3.320278, 1.120433, 4.718157], dtype='float32'),#9
                                np.array([3.390705, 1.119165, 4.717033], dtype='float32'),
                                np.array([3.390214, 1.049049, 4.717045], dtype='float32'),
                                np.array([3.319664, 1.049872, 4.717888], dtype='float32'),
                                np.array([2.392681, 1.950210, 4.731383], dtype='float32'),#10
                                np.array([2.463392, 1.949684, 4.730354], dtype='float32'),
                                np.array([2.463190, 1.879335, 4.729876], dtype='float32'),
                                np.array([2.392331, 1.879559, 4.730327], dtype='float32'),
                                np.array([3.433949, 1.954988, 3.711313], dtype='float32'),#11
                                np.array([3.363037, 1.955199, 3.712410], dtype='float32'),
                                np.array([3.362706, 1.884561, 3.711773], dtype='float32'),
                                np.array([3.433152, 1.884627, 3.710774], dtype='float32'),
                                np.array([3.340848, 1.947487, 4.721658], dtype='float32'),#12
                                np.array([3.411073, 1.948027, 4.721175], dtype='float32'),
                                np.array([3.411450, 1.876894, 4.720297], dtype='float32'),
                                np.array([3.341312, 1.876874, 4.720818], dtype='float32'),
                                np.array([1.607035, 1.225459, 4.733387], dtype='float32'),#13
                                np.array([1.677260, 1.225716, 4.733266], dtype='float32'),
                                np.array([1.676788, 1.155195, 4.733228], dtype='float32'),
                                np.array([1.606296, 1.155602, 4.733340], dtype='float32'),
                                np.array([2.056773, 1.634351, 3.727209], dtype='float32'),#14
                                np.array([2.127354, 1.634262, 3.726511], dtype='float32'),
                                np.array([2.126954, 1.563718, 3.726374], dtype='float32'),
                                np.array([2.056445, 1.563950, 3.727310], dtype='float32'),
                                np.array([2.137509, 1.634158, 3.726697], dtype='float32'),#15
                                np.array([2.207999, 1.634179, 3.726106], dtype='float32'),
                                np.array([2.207560, 1.563553, 3.726026], dtype='float32'),
                                np.array([2.137274, 1.563835, 3.725895], dtype='float32'),
                                np.array([2.686361, 1.857154, 4.734850], dtype='float32'),#16
                                np.array([2.908358, 1.855408, 4.734356], dtype='float32'),
                                np.array([2.906910, 1.633135, 4.732576], dtype='float32'),
                                np.array([2.685181, 1.634751, 4.732922], dtype='float32'),
                                np.array([3.560990, 1.853480, 4.322092], dtype='float32'),#17
                                np.array([3.558040, 1.853335, 4.100673], dtype='float32'),
                                np.array([3.558253, 1.630916, 4.100731], dtype='float32'),
                                np.array([3.560810, 1.631367, 4.321962], dtype='float32'),
                                np.array([1.574309, 1.864690, 4.120169], dtype='float32'),#18
                                np.array([1.578462, 1.861707, 4.340886], dtype='float32'),
                                np.array([1.579109, 1.640250, 4.338705], dtype='float32'),
                                np.array([1.576131, 1.642748, 4.117369], dtype='float32'),
                                np.array([2.742995, 1.859755, 3.707888], dtype='float32'),#19
                                np.array([2.521174, 1.858428, 3.709131], dtype='float32'),
                                np.array([2.522626, 1.636416, 3.708690], dtype='float32'),
                                np.array([2.744045, 1.637673, 3.707437], dtype='float32'),
                                np.array([1.898768, 1.863710, 3.714881], dtype='float32'),#20
                                np.array([1.677394, 1.863392, 3.718481], dtype='float32'),
                                np.array([1.677784, 1.641300, 3.717775], dtype='float32'),
                                np.array([1.899530, 1.641616, 3.713825], dtype='float32'),
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

                                np.array([2.056773, 1.634351+0.13, 3.727209], dtype='float32'),#4 #21
                                np.array([2.127354, 1.634262+0.13, 3.726511], dtype='float32'),#3
                                np.array([2.126954, 1.563718+0.13, 3.726374], dtype='float32'),#2
                                np.array([2.056445, 1.563950+0.13, 3.727310], dtype='float32'),#1
                                np.array([2.137509, 1.634158+0.13, 3.726697], dtype='float32'),#4 #22
                                np.array([2.207999, 1.634179+0.13, 3.726106], dtype='float32'),#3
                                np.array([2.207560, 1.563553+0.13, 3.726026], dtype='float32'),#2
                                np.array([2.137274, 1.563835+0.13, 3.725895], dtype='float32'),#1                                       
                                ]


# Origin of the tag frame expressed in the global metashape frame
origin_global_tag = np.array([2.1207, 1.9766, 4.7421], dtype = 'float32')


# Elementwise subtracion of the origin vector from the tag corners
t_tag_corners_list = [vector - origin_global_tag for vector in t_global_tag_corners_list]


# Create a list of numpy arrays where each array contains four tuples of coordinates from t_tag_corners_list
# Initialize the list
pos_board = []

# Loop through the t_tag_corners_list in steps of 4 up to the 84th element
for i in range(0, 108, 4):  # from 0 to 80 by steps of 4 (to include up to t_tag_corners_list[83])
    # Create an array of four tuples of coordinates and append to pos_board
    pos_board.append(np.array([(t_tag_corners_list[i][0], t_tag_corners_list[i][1], t_tag_corners_list[i][2]), (t_tag_corners_list[i+1][0], t_tag_corners_list[i+1][1], t_tag_corners_list[i+1][2]), (t_tag_corners_list[i+2][0], t_tag_corners_list[i+2][1], t_tag_corners_list[i+2][2]), (t_tag_corners_list[i+3][0], t_tag_corners_list[i+3][1], t_tag_corners_list[i+3][2])], dtype='float32'))

# Now pos_board contains groups of 4 coordinates, from t_tag_corners_list[0] to t_tag_corners_list[83]

# print('Jadda')

#for formatted_array in pos_board:
#    print(formatted_array)

# print(pos_board[20])

# Defines the Ids of the markers in the board
id_board = np.array([[0], [1], [2], [3], [4], [5], [6], [7], [8], [9], [10], [11], [12], [13], [14], [15], [16], [17], [18], [19], [20], [49], [46], [47], [45], [21], [22]])