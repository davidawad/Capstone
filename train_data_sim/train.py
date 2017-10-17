import cv2
import numpy as np
import matplotlib.pylab as plt
import keras
from keras.layers import \
    Dense, Dropout, Activation, Flatten, \
    Convolution2D, MaxPooling2D, Lambda
from keras.models import Sequential
from keras.optimizers import Adam
from keras.callbacks import ModelCheckpoint
from sklearn.utils import shuffle
from sklearn.model_selection import train_test_split
import os
import glob
from imgaug import augmenters as iaa
from keras.utils import np_utils

# model
def create_model():
    model = Sequential()
    model.add(Lambda(lambda x: x/127.5 - 1.0, input_shape=(64,64,3)))
    model.add(Convolution2D(8, 3, 3, activation='relu', input_shape=(64,64,3)))
    model.add(MaxPooling2D(pool_size=(2, 2)))
    model.add(Convolution2D(12, 3, 3, activation='relu'))
    model.add(MaxPooling2D(pool_size=(2, 2), border_mode='same'))
    model.add(Convolution2D(16, 3, 3, activation='relu'))
    model.add(MaxPooling2D(pool_size=(2, 2), border_mode='same'))
    model.add(Flatten())
    model.add(Dense(64, activation='relu'))
    model.add(Dense(16, activation='relu'))
    model.add(Dense(4, activation='softmax'))
    model.compile(loss='categorical_crossentropy', optimizer='adam', metrics=['accuracy'])
    return model

def aug(imgs, max_num, seq):
    n = max_num // len(imgs) - 1
    b = max_num % len(imgs)
    res = imgs
    for _ in range(n):
        aug_imgs = seq.augment_images(imgs)
        res = res + aug_imgs
    for i in range(b):
        img = seq.augment_images(imgs[i])
        res.append(img)
    return res

def tr_generate(red, yellow, green, unknown, n_percat):
    while True:
        yield 1

def val_generate(red, yellow, green, unknown, n_percat):
    while True:
        yield 1

def main():
    red = [cv2.imread(f) for f in glob.glob('*_state_0.jpg')]
    yellow = [cv2.imread(f) for f in glob.glob('*_state_1.jpg')]
    green = [cv2.imread(f) for f in glob.glob('*_state_2.jpg')]
    unknown = [cv2.imread(f) for f in glob.glob('*_state_4.jpg')]

    # augment data
    n_percat = len(red) * 3


    seq = iaa.Sequential([
        iaa.Fliplr(.5),
        iaa.Affine(translate_percent={"x": (-0.2, 0.2), "y": (-0.2, 0.2)}, rotate=(-10, 10)),
        iaa.GaussianBlur(sigma=(0, 1.0))
    ])

    red_aug = aug(red, n_percat, seq)
    yellow_aug = aug(yellow, n_percat, seq)
    green_aug = aug(green, n_percat, seq)   

    noise_seq = iaa.Sequential([
        iaa.Fliplr(.5),
        iaa.AdditiveGaussianNoise(0, scale=(0,255), per_channel=0.8)
    ])
    unknown_aug = aug(unknown, n_percat, noise_seq)

    xs = red_aug + yellow_aug + green_aug + unknown_aug
    ys = [0.]*n_percat + [1.]*n_percat + [2.]*n_percat + [3.]*n_percat    

    xs_tr = np.array(xs)
    ys_tr = np_utils.to_categorical(ys, 4)

    xs_tr, ys_tr = shuffle(xs_tr, ys_tr)

    xs_val = red + yellow + green #+ unknown
    ys_val = [0.]*len(red) + [1.]*len(yellow) + [2.]*len(green) #+ [3.]*len(unknown)
    xs_val = np.array(xs_val)
    ys_val = np_utils.to_categorical(ys_val, 4)

    model = create_model()
    callbacks = [
        keras.callbacks.EarlyStopping(monitor='val_loss',patience=2,verbose=0)
    ]
    model.fit(x=xs_tr, y=ys_tr, batch_size=64, nb_epoch=50, validation_data=(xs_val, ys_val), callbacks=callbacks)
    model.save('tl_model.h5')

if __name__ == '__main__':
    main()