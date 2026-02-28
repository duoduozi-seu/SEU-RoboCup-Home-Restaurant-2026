import face_recognition

image = face_recognition.load_image_file("/home/syw/code/robocup_nation/2024code/GPSR/src/seu_look_for_person/scripts/obama-1080p.jpg")
face_locations = face_recognition.face_locations(image)
for face_location in face_locations:
    # Print the location of each face in this image
    top, right, bottom, left = face_location
    print("A face is located at pixel location Top: {}, Left: {}, Bottom: {}, Right: {}".format(top, left, bottom, right))
    