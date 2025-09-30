cat > README.md << 'EOF'
# SegNetLite: Lightweight Semantic Segmentation Network

A compact implementation of the **SegNet** architecture for semantic segmentation, designed for digit/background segmentation tasks (11 classes: 10 digits + background). This model uses encoder-decoder structure with max-pooling indices for precise upsampling.

## Architecture Overview

SegNetLite follows the classic SegNet design:
- **Encoder (Downsampling)**:  
  `Conv2D → BatchNorm → ReLU → MaxPool (with indices)`
- **Decoder (Upsampling)**:  
  `MaxUnpool (using stored indices) → Conv2D → BatchNorm → ReLU`
- **Final Layer**: 1×1 convolution for 11-class output

### Default Configuration
| Stage | Input Size | Output Size | Filters |
|-------|------------|-------------|---------|
| Input | 3×64×64    | -           | -       |
| Down1 | 3×64×64    | 32×32×32    | 32      |
| Down2 | 32×32×32   | 64×16×16    | 64      |
| Down3 | 64×16×16   | 128×8×8     | 128     |
| Down4 | 128×8×8    | 256×4×4     | 256     |
| Up1   | 256×4×4    | 128×8×8     | 128     |
| Up2   | 128×8×8    | 64×16×16    | 64      |
| Up3   | 64×16×16   | 32×32×32    | 32      |
| Up4   | 32×32×32   | 32×64×64    | 32      |
| Final | 32×64×64   | 11×64×64    | 11      |

## Features

- **Memory-efficient**: Stores only pooling indices (not full feature maps) for upsampling
- **Configurable**: All layer parameters customizable via constructor arguments
- **BatchNorm support**: Includes batch normalization in all conv blocks
- **Flexible input**: Handles any input size compatible with pooling operations
