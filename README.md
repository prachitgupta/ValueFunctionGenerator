# ValueFunctionGenerator

##Steps to get ValueFunctions at a specified point on grid
1) Clone Repo
2) add ValueFunctionGenerator folder to path
3) cd helperOC/
4) [data , tau, ~] = my_brs(gridsize)   **example = [13,13,13,9,9]
5) data = 6d double  last dimension time
6) ValueFunctionArray = data(:,:,:,:,:,end)   **ValueFunctions at every state
7)  value = eval_u(g,ValueFunctionArray,xinit); **replace xinit with your state vector at which you have to obtain value

    eval_u.m can be found under helperOC/ValueFunctions
    scripts I added upon helperOC and LevelSet

     helperOC/Dy
      
