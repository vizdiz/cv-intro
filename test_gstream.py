from video import Video
from matplotlib import pyplot as plt
from time import sleep

video = Video()
print('Initialising stream...')
waited = 0
while not video.frame_available():
    waited += 1
    print('\r  Frame not available (x{})'.format(waited), end='')
    sleep(0.1)
    # cv2.waitKey(30)
    if waited > 100:
        print("failed")
        break
print('\nSuccess!\nStarting streaming - press "q" to quit.')




# Wait for the next frame to become available
if video.frame_available():
    # Only retrieve and display a frame if it's new
    frame = video.frame()
    # cv2.imshow('frame', frame)
# Allow frame to display, and check if user wants to quit
# if cv2.waitKey(1) & 0xFF == ord('q'):
#     break

# plt.imshow(frame)