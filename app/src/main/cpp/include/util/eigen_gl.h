//
// Created by Artem on 15.07.2024.
//

#ifndef TUSA_ANDROID_EIGEN_GL_H
#define TUSA_ANDROID_EIGEN_GL_H

#include <Eigen/Core>
#include <Eigen/StdVector>
#include <Eigen/Dense>

class EigenGL {
public:

    static std::vector<Eigen::Vector4f> extractFrustumPlanes(const Eigen::Matrix4f& matrix) {
        std::vector<Eigen::Vector4f> planes(6);

        // Left Plane
        planes[0] = matrix.row(3) + matrix.row(0);

        // Right Plane
        planes[1] = matrix.row(3) - matrix.row(0);

        // Bottom Plane
        planes[2] = matrix.row(3) + matrix.row(1);

        // Top Plane
        planes[3] = matrix.row(3) - matrix.row(1);

        // Near Plane
        planes[4] = matrix.row(3) + matrix.row(2);

        // Far Plane
        planes[5] = matrix.row(3) - matrix.row(2);

        return planes;
    }

    static Eigen::Matrix4f createOrthoMatrix(float left, float right, float bottom, float top, float nearVal, float farVal) {
        Eigen::Matrix4f orthoMatrix = Eigen::Matrix4f::Identity();

        orthoMatrix(0, 0) = 2.0f / (right - left);
        orthoMatrix(1, 1) = 2.0f / (top - bottom);
        orthoMatrix(2, 2) = -2.0f / (farVal - nearVal);

        orthoMatrix(0, 3) = -(right + left) / (right - left);
        orthoMatrix(1, 3) = -(top + bottom) / (top - bottom);
        orthoMatrix(2, 3) = -(farVal + nearVal) / (farVal - nearVal);

        return orthoMatrix;
    }

    static Eigen::Vector2f rotateVector(const Eigen::Vector2f& vec, float angleRad) {
        // Convert angle from degrees to radians

        // Create the rotation matrix
        Eigen::Matrix2f rotationMatrix;
        rotationMatrix << cos(angleRad), -sin(angleRad),
                sin(angleRad),  cos(angleRad);

        // Rotate the vector
        return rotationMatrix * vec;
    }

    static Eigen::Matrix4f createTranslationMatrix(float tx, float ty, float tz) {
        // Start with an identity matrix
        Eigen::Matrix4f translationMatrix = Eigen::Matrix4f::Identity();

        // Set the translation components
        translationMatrix(0, 3) = tx;
        translationMatrix(1, 3) = ty;
        translationMatrix(2, 3) = tz;

        return translationMatrix;
    }

    static Eigen::Matrix4f createPerspectiveProjectionMatrix(float fovY, float aspectRatio, float near, float far) {
        // Convert field of view from degrees to radians
        float fovYRad = fovY * (M_PI / 180.0f);

        // Calculate the scale factors
        float tanHalfFovy = tan(fovYRad / 2.0f);

        Eigen::Matrix4f projectionMatrix1 = Eigen::Matrix4f::Zero();

        projectionMatrix1(0, 0) = 1.0f / (aspectRatio * tanHalfFovy);
        projectionMatrix1(1, 1) = 1.0f / tanHalfFovy;
        projectionMatrix1(2, 2) = -(far + near) / (far - near);
        projectionMatrix1(2, 3) = -(2.0f * far * near) / (far - near);
        projectionMatrix1(3, 2) = -1.0f;

        return projectionMatrix1;
    }

    static Eigen::Matrix4f createViewMatrix(const Eigen::Vector3f& cameraPos, const Eigen::Vector3f& target, const Eigen::Vector3f& up) {
        // Calculate forward, right, and up vectors
        Eigen::Vector3f forward = (cameraPos - target).normalized();
        Eigen::Vector3f right = up.cross(forward).normalized();
        Eigen::Vector3f cameraUp = forward.cross(right).normalized();

        // Create a 4x4 view matrix
        Eigen::Matrix4f viewMatrix = Eigen::Matrix4f::Identity();

        // Set rotation part (top-left 3x3)
        viewMatrix(0, 0) = right.x();
        viewMatrix(0, 1) = right.y();
        viewMatrix(0, 2) = right.z();
        viewMatrix(1, 0) = cameraUp.x();
        viewMatrix(1, 1) = cameraUp.y();
        viewMatrix(1, 2) = cameraUp.z();
        viewMatrix(2, 0) = forward.x();
        viewMatrix(2, 1) = forward.y();
        viewMatrix(2, 2) = forward.z();

        // Set translation part (top-right 3x1)
        viewMatrix(0, 3) = -right.dot(cameraPos);
        viewMatrix(1, 3) = -cameraUp.dot(cameraPos);
        viewMatrix(2, 3) = -forward.dot(cameraPos);

        return viewMatrix;
    }
};


#endif //TUSA_ANDROID_EIGEN_GL_H
