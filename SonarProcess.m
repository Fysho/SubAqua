classdef SonarProcess < handle
    %UR3 Class for UR3 robot simulation

    properties
        threshold = 0.65;
        imageres = 400;
        kernel1 = [0.1 0.1 0.1; 0.1 0.2 0.1; 0.1 0.1 0.1];
        kernel2 = [0.05 0.125 0.05; 0.125 0.3 0.125; 0.05 0.125 0.05];
        kernel3 = [0.05 0.125 0.05; 0.125 0.3 0.125; 0.05 0.125 0.05];
        kernel4 = [0.05 0.05 0.05 0.05 0.05;
                   0.05 0.05 0.05 0.05 0.05;
                   0.05 0.05 0.15 0.05 0.05;
                   0.05 0.05 0.05 0.05 0.05;
                   0.05 0.05 0.05 0.05 0.05];
        kernel5 = [1];
    end
    
    methods
        function self = SonarProcess(threshold)
            self.threshold = threshold;
            
        end
        
        function heatmap = CreateHeatMap(self, image)
            steps = size(image,1);
            samplesPerStep = size(image,2);
            heatmap = zeros(self.imageres,self.imageres,3,'uint8');
            angle = 0;
            
            for step = 1:steps
                for sample = 1:samplesPerStep
                    distanceOut = (((self.imageres/2.0) / samplesPerStep)  * sample) * 0.98;
                    x = self.imageres/2.0 + sin(angle) * distanceOut;
                    y = self.imageres/2.0 - cos(angle) * distanceOut;

                    colour = image(step, sample);
                    
                    xpix = round(x);
                    ypix = round(y);

                    heatmap(ypix-1,xpix-1,:) = colour;
                    heatmap(ypix-1,xpix,  :) = colour;
                    heatmap(ypix-1,xpix+1,:) = colour;
                    heatmap(ypix,  xpix-1,:) = colour;
                    heatmap(ypix,  xpix,  :) = colour;
                    heatmap(ypix,  xpix+1,:) = colour;
                    heatmap(ypix+1,xpix-1,:) = colour;
                    heatmap(ypix+1,xpix,  :) = colour;
                    heatmap(ypix+1,xpix+1,:) = colour;
                end
                angle = angle + deg2rad(360.0/steps);
            end
%             SampleImage(imageRes,imageRes,:) = [0,0,0];

%             imshow(SampleImage);
%             heatmap = image;
        end
        
        function flatImage = ConvolveImage(self, sonarData, k)
            image = sonarData / 255;
            kernel = self.kernel5;
            if(k == 1) 
                kernel = self.kernel1;
            elseif (k == 2)
                kernel = self.kernel2;
            elseif (k == 3)
                kernel = self.kernel3;
            elseif (k == 4)
                kernel = self.kernel4;
            elseif (k == 5)
                kernel = self.kernel5;
            end        
                
            convolvedImage = conv2(kernel, image);
%             imshow(convolvedImage);
            flatImage = convolvedImage > self.threshold;
        end
    end
end