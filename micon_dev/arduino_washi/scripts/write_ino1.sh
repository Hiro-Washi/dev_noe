echo "\nWriting to arduino\nConnect arduino on USB!"

cd `dirname ${0}`/../platformio/ino01
platformio run

