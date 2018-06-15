<<<<<<< HEAD
# Pong

> Author: ~~Leslie Lee([LeeChunHei](https://github.com/LeeChunHei))~~ 

<b>🎈🎊🎉<i>Happy New Year!!! Your warm and friendly seniors have brought you all a present, this lovely little puzzle for you to solve with your friends!</i>🎉🎊🎈</b>

Mixed advance software, libbase and libsc assignment for Smart Car 2018.

Please thank Leslie for this assignment. All are designed by Leslie.

### Introduction

The Pong assignment requires a design of an MCU program which can play the (slightly modified) classic game Pong between 2 people each using an internal board with various components listed below. The game synchronizes wirelessly between the 2 players via Bluetooth. The software must follow the specified communication protocols to ensure consistency between the devices. Since one group only has one single board, at least 2 groups are needed to complete the task together for testing out communication.

Detailed description: https://docs.google.com/document/d/1o12wKFjEMJ87Q3X8X253FEP3rcGSzep563lBsyIKvwo/

### Skeleton Code

As boy god Leslie knows you are not as god as him, he will provide some skeleton code, you can feel free to use/modify or even choose not using it. You can also add or delete files freely, as long as it can give the correct output. The headers are not necessarily complete, you may need to further modify them for your convenience.

Provided skeletons:

- `config.h`: stores all components' configs.
- `comm.h`: class prototype of communication protocol abstract class `Comm`.
- `bluetooth.h` : prototype of `Bluetooth` class inherited from `Comm`, implement the pure virtual function, which `SendBuffer` should become `JyMcuBt106`'s and `EnableTimer` should be enabling `Pit`.
- `sprite.h`: prototype of the `Sprite` abstract class for managing on-screen elements. This class hints the way you might want to follow for rendering the elements.
- `ball.h`: prototype of the `Ball` class.
- `platform.h`: prototype of the `Platform` class.
- `main.cpp`: the program which seniors use for testing protocol. Pay attention to the lambda function syntax, you may want to use it 😈. 

### Submission

To submit, set up a private Github repo and invite all spectators as collaborators. You need to finish your protocol before 5 Jan 2018, and mark it as Milestone 1 on Github; finish the whole game before 10 Jan 2018, and mark it as Milestone 2. There will be a tournament later, date, time, venue TBA.

All spectators:

-   LeeChunHei
-   mcreng
-   danvim
-   Mkwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwwww
-   dipsywong98

### Getting Started

1. Set up an eclipse project for development on the board.
2. Run the following command in your project directory.

```shell
git init
git remote add origin https://github.com/hkust-smartcar/assignment-pong
git pull origin master
git remote rm origin
git remote add origin https://github.com/<your user id>/<repo-name>
git add *
git commit -a -m "initial commit"
git push --set-upstream origin master
```

=======
<<<<<<< HEAD
# Smart-Car-Internal
=======
# Dual18
2018 Dual Car Group
>>>>>>> f717e60cb3bb53fa54abacd1b676ebb61f135aa7
>>>>>>> morris_branch
