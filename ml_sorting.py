import pickle

with open("robot_brain.pkl", "rb") as file:
    tree_model = pickle.load(file)


def ml_sorting(ball_hue, ball_radius):
    """
    Takes data from the CV camera and decides which bin to use.
    0 = Baseball (Bin 1)
    1 = Tennis Ball (Bin 2)
    2 = Ping Pong Ball (Bin 3)
    """
    guess = tree_model.predict([[ball_hue, ball_radius]])[0]
    return int(guess)