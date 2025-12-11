import type {ReactNode} from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Layout from '@theme/Layout';
import HomepageFeatures from '@site/src/components/HomepageFeatures';
import Heading from '@theme/Heading';

import styles from './index.module.css';

function HomepageHeader() {
  const {siteConfig} = useDocusaurusContext();
  return (
    <header className={clsx('hero', styles.heroBanner)} role="banner">
      <div className="container">
        <div className={styles.heroContent} role="presentation">
          <div className={styles.heroText}>
            <div className={styles.badge} aria-label="Robotics theme tag">
              <span className={styles.badgeDot} aria-hidden="true"></span>
              Build the Future of Robotics
            </div>
            <Heading as="h1" className={styles.heroTitle} id="main-title">
              Master AI-Native Robotics
            </Heading>
            <p className={styles.heroSubtitle} id="main-description">
              A comprehensive, hands-on textbook for building intelligent robots using ROS2,
              Digital Twins, NVIDIA Isaac, and Vision-Language-Action models.
            </p>
            <div className={styles.heroButtons} role="group" aria-label="Main navigation options">
              <Link
                className="button button--primary button--lg"
                to="#weeks-plan"
                aria-label="View the 13-week learning plan">
                Start Learning
              </Link>
              <Link
                className="button button--outline button--secondary button--lg"
                to="/docs/chapter1/physical-ai"
                aria-label="Explore book chapters">
                Explore Chapters
              </Link>
            </div>
            <div className={styles.heroStats} role="group" aria-label="Course statistics">
              <div className={styles.stat}>
                <div className={styles.statNumber} aria-label="Number of chapters">5</div>
                <div className={styles.statLabel}>Chapters</div>
              </div>
              <div className={styles.stat}>
                <div className={styles.statNumber} aria-label="Number of tutorials">50+</div>
                <div className={styles.statLabel}>Tutorials</div>
              </div>
              <div className={styles.stat}>
                <div className={styles.statNumber} aria-label="Open source percentage">100%</div>
                <div className={styles.statLabel}>Open Source</div>
              </div>
            </div>
          </div>
          <div className={styles.heroImage} role="figure" aria-label="Robotic illustration">
            <img
              src="/ai-native-book/img/hero-robot.svg"
              alt="AI-Native Robotics Illustration"
              className={styles.heroRobot}
              loading="eager"
            />
          </div>
        </div>
      </div>
      <div className={styles.heroWave} aria-hidden="true">
        <svg viewBox="0 0 1440 120" xmlns="http://www.w3.org/2000/svg">
          <path
            fill="currentColor"
            fillOpacity="0.1"
            d="M0,64L48,69.3C96,75,192,85,288,80C384,75,480,53,576,48C672,43,768,53,864,58.7C960,64,1056,64,1152,58.7C1248,53,1344,43,1392,37.3L1440,32L1440,120L1392,120C1344,120,1248,120,1152,120C1056,120,960,120,864,120C768,120,672,120,576,120C480,120,384,120,288,120C192,120,96,120,48,120L0,120Z"
          />
        </svg>
      </div>
    </header>
  );
}

function WeeksPlan() {
  const weeks = [
    { week: '1-2', title: 'Foundations', topics: 'Physical AI concepts, Python refresher, Development environment setup' },
    { week: '3-4', title: 'ROS2 Basics', topics: 'Nodes, Topics, Services, Launch files, Basic robot control' },
    { week: '5-6', title: 'Sensors & Perception', topics: 'Camera integration, LiDAR processing, Sensor fusion' },
    { week: '7-8', title: 'Simulation & Digital Twins', topics: 'Gazebo setup, URDF modeling, Sim-to-real transfer' },
    { week: '9-10', title: 'NVIDIA Isaac Platform', topics: 'Isaac Sim, Isaac ROS packages, GPU acceleration' },
    { week: '11-12', title: 'AI Integration', topics: 'Vision-Language-Action models, Multimodal learning' },
    { week: '13', title: 'Capstone Project', topics: 'Build and deploy your own intelligent robot system' },
  ];

  return (
    <section className={styles.weeksPlan} id="weeks-plan" aria-labelledby="weeks-plan-title">
      <div className="container">
        <div className={styles.sectionHeader}>
          <Heading as="h2" id="weeks-plan-title">13-Week Learning Plan</Heading>
          <p>A structured roadmap to master AI-Native Robotics from scratch</p>
        </div>
        <div className={styles.weeksGrid} role="list">
          {weeks.map((item, idx) => (
            <div key={idx} className={styles.weekCard} role="listitem">
              <div className={styles.weekNumber}>Week {item.week}</div>
              <h3 className={styles.weekTitle}>{item.title}</h3>
              <p className={styles.weekTopics}>{item.topics}</p>
            </div>
          ))}
        </div>
      </div>
    </section>
  );
}

function TechnologyStack() {
  const technologies = [
    { name: 'ROS2', description: 'Robot Operating System 2', icon: '🤖' },
    { name: 'Python', description: 'Primary Language', icon: '🐍' },
    { name: 'Gazebo', description: 'Physics Simulation', icon: '🔬' },
    { name: 'NVIDIA Isaac', description: 'AI Acceleration', icon: '⚡' },
    { name: 'PyTorch', description: 'Deep Learning', icon: '🔥' },
    { name: 'Docker', description: 'Containerization', icon: '🐳' },
  ];

  return (
    <section className={styles.techStack} aria-labelledby="tech-stack-title">
      <div className="container">
        <div className={styles.sectionHeader}>
          <Heading as="h2" id="tech-stack-title">Built on Industry-Leading Technologies</Heading>
          <p>Learn the tools that power modern robotics companies</p>
        </div>
        <div className={styles.techGrid} role="list">
          {technologies.map((tech, idx) => (
            <div key={idx} className={styles.techCard} role="listitem">
              <div className={styles.techIcon} aria-hidden="true">{tech.icon}</div>
              <div className={styles.techName} id={`tech-${idx}-name`}>{tech.name}</div>
              <div className={styles.techDescription}
                   aria-describedby={`tech-${idx}-name`}>
                {tech.description}
              </div>
            </div>
          ))}
        </div>
      </div>
    </section>
  );
}

function LearningPath() {
  const chapters = [
    {
      number: '01',
      title: 'Physical AI Fundamentals',
      description: 'Understand embodied intelligence and the transition from digital to physical AI',
      topics: ['Digital to Physical', 'Sensor Systems', 'Real-time Processing'],
    },
    {
      number: '02',
      title: 'ROS2 - The Robotic Nervous System',
      description: 'Master the middleware that powers modern robots',
      topics: ['Nodes & Topics', 'Services & Actions', 'Launch Systems'],
    },
    {
      number: '03',
      title: 'Digital Twins & Simulation',
      description: 'Build and test robots in virtual environments before deployment',
      topics: ['Gazebo', 'URDF Models', 'Sim-to-Real Transfer'],
    },
    {
      number: '04',
      title: 'NVIDIA Isaac Platform',
      description: 'Leverage GPU-accelerated perception and navigation',
      topics: ['Isaac Sim', 'Isaac ROS', 'Synthetic Data'],
    },
    {
      number: '05',
      title: 'Vision-Language-Action Models',
      description: 'Deploy foundation models for intelligent robot control',
      topics: ['Multimodal Learning', 'Action Prediction', 'Natural Language Commands'],
    },
  ];

  return (
    <section className={styles.learningPath} aria-labelledby="learning-path-title">
      <div className="container">
        <div className={styles.sectionHeader}>
          <Heading as="h2" id="learning-path-title">Your Learning Journey</Heading>
          <p>A structured path from fundamentals to advanced AI robotics</p>
        </div>
        <div className={styles.timeline} role="list">
          {chapters.map((chapter, idx) => (
            <div key={idx} className={styles.timelineItem} role="listitem">
              <div className={styles.timelineMarker}>
                <div
                  className={styles.chapterNumber}
                  aria-label={`Chapter ${chapter.number}`}>
                  {chapter.number}
                </div>
              </div>
              <div className={styles.timelineContent}>
                <h3>{chapter.title}</h3>
                <p>{chapter.description}</p>
                <div className={styles.topicTags} aria-label="Chapter topics">
                  {chapter.topics.map((topic, i) => (
                    <span key={i} className={styles.topicTag}>{topic}</span>
                  ))}
                </div>
              </div>
            </div>
          ))}
        </div>
      </div>
    </section>
  );
}

function CallToAction() {
  return (
    <section className={styles.cta} aria-labelledby="cta-title">
      <div className="container">
        <div className={styles.ctaContent}>
          <Heading as="h2" id="cta-title">Ready to Build Intelligent Robots?</Heading>
          <p>
            Join thousands of students, researchers, and engineers learning to build
            the next generation of AI-powered robotic systems.
          </p>
          <div className={styles.ctaButtons} role="group" aria-label="Final call to action">
            <Link
              className="button button--primary button--lg"
              to="/docs/intro"
              aria-label="Start your robotics learning journey now">
              Start Your Journey
            </Link>
            <Link
              className="button button--outline button--secondary button--lg"
              href="https://github.com/AsmaIqbal01/ai-native-book"
              aria-label="View project on GitHub">
              View on GitHub
            </Link>
          </div>
        </div>
      </div>
    </section>
  );
}

export default function Home(): ReactNode {
  const {siteConfig} = useDocusaurusContext();
  return (
    <Layout
      title="Learn AI-Native Robotics"
      description="A comprehensive textbook for building intelligent robots using ROS2, Digital Twins, NVIDIA Isaac, and Vision-Language-Action models. Hands-on, practical, and industry-focused.">
      <HomepageHeader />
      <main>
        <HomepageFeatures />
        <WeeksPlan />
        <TechnologyStack />
        <LearningPath />
        <CallToAction />
      </main>
    </Layout>
  );
}
