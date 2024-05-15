import torch
from torchvision import models, transforms
from PIL import Image
import numpy as np
import torch.nn as nn

# Define class labels
class_labels = {
    0: "Earth",
    1: "Jupiter",
    2: "MakeMake",
    3: "Mars",
    4: "Mercury",
    5: "Moon",
    6: "Neptune",
    7: "Pluto",
    8: "Saturn",
    9: "Uranus",
    10: "Venus"
}

model_path = 'src/group-project-group-5/group_project/detect_planets.py'

#loading and preparing model
def load_model(model_path):
    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    model = models.vgg16(pretrained=True)
    model.classifier[6] = nn.Linear(4096, 11)  #adjusting the classifier for 11 classes
    model.load_state_dict(torch.load(model_path, map_location=device))
    model = model.to(device)
    model.eval()  #setting the model to evaluation mode
    return model

model = load_model(model_path) #loading the model

# Function to detect planets from an image input
def detect_planets(image_path):
    transform = transforms.Compose([
        transforms.Resize((256, 256)),
        transforms.ToTensor(),
        transforms.Normalize(mean=[0.485, 0.456, 0.406], std=[0.229, 0.224, 0.225])
    ])

    # loading image and applying transformations
    img = Image.open(image_path).convert('RGB')
    img = transform(img).unsqueeze(0)  # Add batch dimension
    img = img.to(torch.device("cuda" if torch.cuda.is_available() else "cpu"))

    with torch.no_grad():
        outputs = model(img)
        probabilities = torch.nn.functional.softmax(outputs, dim=1)
        probabilities = probabilities.cpu().numpy()[0]

    #storing all probabilities in case this is needed
    all_probabilities = {class_labels[i]: prob * 100 for i, prob in enumerate(probabilities)}

    #finding the most likely planet
    max_index = np.argmax(probabilities)
    detected_planet = class_labels[max_index]

    # Print detected planet and probability for reference
    print(f"Detected Planet: {detected_planet}, Probability: {probabilities[max_index] * 100:.2f}%")

    return detected_planet

# Example usage
'''
if __name__ == "__main__":
    image_path = '/path/to/your/image.jpg'  # Update this path as necessary
    detected_planet = detect_planets(image_path)
    print(f"Detected Planet: {detected_planet}")
'''
