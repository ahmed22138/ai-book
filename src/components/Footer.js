import React from 'react';
import styles from './Footer.module.css';

export default function Footer() {
  return (
    <footer className={styles.footer}>
      <div className={styles.footerContent}>
        <div className={styles.footerGrid}>
          {/* Brand Section */}
          <div className={styles.footerSection}>
            <div className={styles.footerLogo}>🤖</div>
            <h3>AI Textbook</h3>
            <p className={styles.description}>
              Master embodied intelligence through interactive, hands-on learning with real-world robotics examples.
            </p>
            <div className={styles.socialLinks}>
              <a href="https://github.com/your-username/ai-textbook" title="GitHub" className={styles.socialLink}>
                🔗
              </a>
              <a href="https://twitter.com" title="Twitter" className={styles.socialLink}>
                𝕏
              </a>
              <a href="https://linkedin.com" title="LinkedIn" className={styles.socialLink}>
                💼
              </a>
            </div>
          </div>

          {/* Course Section */}
          <div className={styles.footerSection}>
            <h3>Course</h3>
            <ul>
              <li>
                <a href="/ai-textbook/docs/intro">Introduction</a>
              </li>
              <li>
                <a href="/ai-textbook/docs/introduction/week-1-embodied-ai">Week 1: Embodied AI</a>
              </li>
              <li>
                <a href="/ai-textbook/docs">All Modules</a>
              </li>
              <li>
                <a href="/ai-textbook/docs/perception/week-4-computer-vision">Perception & Sensing</a>
              </li>
              <li>
                <a href="/ai-textbook/docs/control/week-7-path-planning">Control & Actuation</a>
              </li>
            </ul>
          </div>

          {/* Resources Section */}
          <div className={styles.footerSection}>
            <h3>Resources</h3>
            <ul>
              <li>
                <a href="https://github.com/your-username/ai-textbook">GitHub Repository</a>
              </li>
              <li>
                <a href="/ai-textbook/docs">Hardware Requirements</a>
              </li>
              <li>
                <a href="/ai-textbook/docs">Setup Guide</a>
              </li>
              <li>
                <a href="/ai-textbook/docs">FAQ</a>
              </li>
            </ul>
          </div>

          {/* About Section */}
          <div className={styles.footerSection}>
            <h3>About</h3>
            <ul>
              <li>
                <a href="/ai-textbook/docs/intro">Documentation</a>
              </li>
              <li>
                <a href="/ai-textbook/docs">Full Curriculum</a>
              </li>
              <li>
                <a href="/ai-textbook/docs/introduction/week-1-embodied-ai">Week 1 Overview</a>
              </li>
              <li>
                <a href="https://github.com/your-username/ai-textbook/issues">Contact</a>
              </li>
            </ul>
          </div>
        </div>

        <div className={styles.footerDivider}></div>

        {/* Bottom Section */}
        <div className={styles.footerBottom}>
          <p>✨ Built with Claude Code | Spec-Kit Plus | AI-Native Textbook</p>
          <p style={{ marginTop: '15px' }}>
            <strong>Status:</strong> 🟢 PRODUCTION READY
          </p>
          <p className={styles.copyright}>
            Copyright © {new Date().getFullYear()} Physical AI & Humanoid Robotics. All rights reserved.
          </p>
          <p style={{ marginTop: '15px', fontSize: '0.8rem', color: '#555' }}>
            Powered by{' '}
            <a href="https://docusaurus.io" style={{ color: '#00d4ff' }}>
              Docusaurus
            </a>
            {' '} | Designed with{' '}
            <span style={{ color: '#ff6b6b' }}>❤️</span>
          </p>
        </div>
      </div>
    </footer>
  );
}
