import React, { useState, useEffect } from 'react';
import clsx from 'clsx';
import styles from './InteractiveContent.module.css';
import { saveNotesToAPI, getNotesFromAPI, updateProgressAPI } from '../services/api';

const InteractiveContent = ({ content, className, userId = 'default-user' }) => {
  const [activeTab, setActiveTab] = useState(0);
  const [bookmarked, setBookmarked] = useState(false);
  const [notes, setNotes] = useState('');
  const [loading, setLoading] = useState(true);
  const [progress, setProgress] = useState(content.progress || 0);

  useEffect(() => {
    const fetchNotes = async () => {
      try {
        const savedNotes = await getNotesFromAPI(userId, content.id);
        if (savedNotes && savedNotes.notes) {
          setNotes(savedNotes.notes);
        }
      } catch (error) {
        console.error('Error fetching notes:', error);
        // Continue with empty notes if fetch fails
      } finally {
        setLoading(false);
      }
    };

    if (content.id) {
      fetchNotes();
    }
  }, [userId, content.id]);

  const tabs = [
    { id: 'content', label: 'Content' },
    { id: 'examples', label: 'Examples' },
    { id: 'exercises', label: 'Exercises' },
  ];

  const handleBookmark = () => {
    setBookmarked(!bookmarked);
  };

  const handleNoteChange = (e) => {
    setNotes(e.target.value);
  };

  const handleSaveNotes = async () => {
    if (!content.id) return;

    try {
      await saveNotesToAPI(userId, content.id, notes);
      alert('Notes saved successfully!');
    } catch (error) {
      console.error('Error saving notes:', error);
      alert('Error saving notes. Please try again.');
    }
  };

  const handleClearNotes = () => {
    setNotes('');
  };

  const updateContentProgress = async (newProgress) => {
    if (!content.id) return;

    try {
      await updateProgressAPI(userId, content.id, newProgress);
      setProgress(newProgress);
    } catch (error) {
      console.error('Error updating progress:', error);
      // Update local state anyway, since API might fail
      setProgress(newProgress);
    }
  };

  return (
    <div className={clsx(styles.interactiveContainer, className)}>
      <div className={styles.header}>
        <h2>{content.title}</h2>
        <div className={styles.actions}>
          <button
            onClick={handleBookmark}
            className={clsx(styles.actionButton, bookmarked && styles.bookmarked)}
          >
            {bookmarked ? '★' : '☆'} Bookmark
          </button>
          <button className={styles.actionButton}>
            <span>⎙</span> Print
          </button>
          <button className={styles.actionButton}>
            <span>-share-</span> Share
          </button>
        </div>
      </div>

      <div className={styles.tabs}>
        {tabs.map((tab, index) => (
          <button
            key={tab.id}
            className={clsx(
              styles.tabButton,
              activeTab === index && styles.activeTab
            )}
            onClick={() => setActiveTab(index)}
          >
            {tab.label}
          </button>
        ))}
      </div>

      <div className={styles.tabContent}>
        {activeTab === 0 && (
          <div className={styles.contentTab}>
            <div className={styles.contentText}>
              {content.description && <p>{content.description}</p>}
              {content.keyPoints && (
                <div className={styles.keyPoints}>
                  <h4>Key Points:</h4>
                  <ul>
                    {content.keyPoints.map((point, index) => (
                      <li key={index}>{point}</li>
                    ))}
                  </ul>
                </div>
              )}
              {content.content && <div>{content.content}</div>}
            </div>

            {content.codeExamples && (
              <div className={styles.codeSection}>
                <h4>Code Examples:</h4>
                {content.codeExamples.map((example, index) => (
                  <div key={index} className={styles.codeBlock}>
                    <div className={styles.codeHeader}>
                      <span className={styles.fileName}>{example.fileName}</span>
                      <button className={styles.copyButton}>Copy</button>
                    </div>
                    <pre className={styles.code}>
                      <code>{example.code}</code>
                    </pre>
                  </div>
                ))}
              </div>
            )}
          </div>
        )}

        {activeTab === 1 && (
          <div className={styles.examplesTab}>
            <h3>Practical Examples</h3>
            {content.examples && content.examples.map((example, index) => (
              <div key={index} className={styles.exampleItem}>
                <h4>{example.title}</h4>
                <p>{example.description}</p>
                {example.code && (
                  <pre className={styles.exampleCode}>
                    <code>{example.code}</code>
                  </pre>
                )}
                {example.simulation && (
                  <div className={styles.simulationContainer}>
                    <h5>Simulation:</h5>
                    <div className={styles.simulationPlaceholder}>
                      {example.simulation}
                    </div>
                  </div>
                )}
              </div>
            ))}
          </div>
        )}

        {activeTab === 2 && (
          <div className={styles.exercisesTab}>
            <h3>Practice Exercises</h3>
            {content.exercises && content.exercises.map((exercise, index) => (
              <div key={index} className={styles.exerciseItem}>
                <h4>{exercise.title}</h4>
                <p>{exercise.description}</p>
                {exercise.difficulty && (
                  <div className={styles.difficulty}>
                    Difficulty: <span className={styles.difficultyLevel}>{exercise.difficulty}</span>
                  </div>
                )}
                <div className={styles.exerciseActions}>
                  <button className={clsx(styles.actionButton, styles.primary)}>
                    Start Exercise
                  </button>
                  <button className={styles.actionButton}>
                    View Solution
                  </button>
                </div>
              </div>
            ))}
          </div>
        )}
      </div>

      <div className={styles.notesSection}>
        <h4>Your Notes</h4>
        <textarea
          value={notes}
          onChange={handleNoteChange}
          placeholder="Add your notes here..."
          className={styles.notesInput}
          rows={4}
        />
        <div className={styles.notesActions}>
          <button
            className={clsx(styles.actionButton, styles.primary)}
            onClick={handleSaveNotes}
          >
            Save Notes
          </button>
          <button
            className={styles.actionButton}
            onClick={handleClearNotes}
          >
            Clear
          </button>
        </div>
      </div>

      <div className={styles.progressSection}>
        <div className={styles.progressBar}>
          <div
            className={styles.progressFill}
            style={{ width: `${progress}%` }}
          ></div>
        </div>
        <div className={styles.progressText}>
          {progress}% Complete
        </div>
      </div>
    </div>
  );
};

// Example usage data
InteractiveContent.defaultProps = {
  content: {
    title: "Introduction to Physical AI",
    description: "Physical AI combines artificial intelligence with physical systems to create robots that can interact with the real world effectively.",
    keyPoints: [
      "Physical AI integrates perception, reasoning, and action",
      "Key components include sensors, actuators, and control systems",
      "Applications span from industrial automation to humanoid robotics"
    ],
    content: "Physical AI represents a paradigm shift in robotics, moving beyond traditional programmed behaviors to adaptive, learning systems. Unlike classical robotics that relies on pre-programmed responses, Physical AI systems can perceive their environment, reason about situations, and take appropriate actions in real-time.",
    codeExamples: [
      {
        fileName: "physical_ai_basic.py",
        code: `import numpy as np
import rospy
from sensor_msgs.msg import JointState

class PhysicalAIController:
    def __init__(self):
        self.joint_sub = rospy.Subscriber('/joint_states', JointState, self.joint_callback)
        self.command_pub = rospy.Publisher('/joint_commands', JointState, queue_size=10)

    def joint_callback(self, msg):
        # Process joint states for physical interaction
        pass`
      }
    ],
    examples: [
      {
        title: "Adaptive Grasping",
        description: "A robot learning to grasp objects of different shapes and sizes using tactile feedback.",
        code: "# Example code for adaptive grasping\n# Implementation details here",
        simulation: "Simulation of robot hand adapting to object shape"
      },
      {
        title: "Balance Control",
        description: "A humanoid robot maintaining balance while walking on uneven terrain.",
        code: "# Example code for balance control\n# Implementation details here",
        simulation: "Simulation of bipedal locomotion control"
      }
    ],
    exercises: [
      {
        title: "Basic Sensor Integration",
        description: "Implement a basic sensor fusion algorithm to combine data from multiple sensors.",
        difficulty: "Beginner"
      },
      {
        title: "Adaptive Control",
        description: "Design an adaptive controller that adjusts parameters based on environmental changes.",
        difficulty: "Intermediate"
      }
    ],
    progress: "45%"
  }
};

export default InteractiveContent;